#!/usr/bin/env python

import rospy
import math
import numpy as np
import pylab as pl
from std_msgs.msg import Float64MultiArray
from scipy.interpolate import interp1d
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# class State:
#     def __init__(self, x, y, z, theta_x, theta_y, theta_z):
#         self.x = x
#         self.y = y
#         self.z = z
#         self.theta_x = theta_x
#         self.theta_y = theta_y
#         self.theta_z = theta_z

def normalizedHeadingRad(theta_z):
    if theta_z < 0:
        theta_z = theta_z - 2.0 * math.pi * (int)(theta_z / (2.0 * math.pi))
        return 2.0 * math.pi + theta_z

    return theta_z - 2.0 * math.pi * (int)(theta_z / (2.0 * math.pi))

def f(s, t, v, omega_z):
    c_x = math.cos(s[3])
    s_x = math.sin(s[3])
    c_y = math.cos(s[4])
    s_y = math.sin(s[4])
    c_z = math.cos(s[5])
    s_z = math.sin(s[5])
    mat_1 = np.matrix(np.array([[c_z*c_y, c_z*s_y*s_x-s_z*c_x, 0.0     ],
                                [s_z*c_y, s_z*s_y*s_x-c_z*c_x, 0.0     ],
                                [0.0,     0.0,                 c_x/c_y]]))
    mat_2 = np.matrix(np.array([v, 0.0, omega_z]))
    return np.array(mat_1 * mat_2.T)



def t_to_v(t, feature):
    v_0 = feature.data[6]
    a_0 = feature.data[7]
    v_tra = feature.data[8]
    a_f = feature.data[9]
    v_f = feature.data[10]
    t_f = feature.data[11]

    t_1 = (v_tra - v_0) / a_0
    t_2 = t_f - ((v_f - v_tra) / a_f)
    if t <= t_1:
        t = max(0, t)
        return v_0 + a_0 * t
    elif t <= t_2:
        return v_tra
    else:
        t = min(t, t_f)
        return v_tra + a_f * (t - t_2)



def callback(features):
    print('I get some features!')
    ## division
    offset = features.layout.data_offset
    fea_num = int(len(features.data) / offset + 0.5)
    fea_list = []
    for i in range(fea_num):
        fea_list.append(list(features.data[i*offset:(i+1)*offset-1]))
    # print(fea_list)
    whole_path = Path()
    whole_path.header.frame_id = 'path'
    for fea in fea_list:
        feature = Float64MultiArray()
        feature.data = fea
        path = calc_predicted_path(feature)
        whole_path.poses += path.poses
    pub.publish(whole_path)



def calc_predicted_path(feature):
    t_f = feature.data[11]
    t_list = []
    omega_z_list = []
    K = len(feature.data) - 12
    Delta_t = t_f / (K - 1)
    for k in range(K):
        t_list.append(k * Delta_t)

    for i in range(12, len(feature.data), 1):
        omega_z_list.append(feature.data[i])

    # print(t_list)
    # print(omega_z_list)
    t_to_omega_z = interp1d(t_list, omega_z_list, kind='cubic')
    # t_new = np.linspace(0, t_f - 0.01, 1000)
    # omega_z_new = t_to_omega_z(t_new)
    # pl.plot(t_new, omega_z_new)

    h = rospy.get_param('/hybrid_astar/dt') # s
    t = 0
    s = [feature.data[0], feature.data[1], feature.data[2],
        feature.data[3], feature.data[4], feature.data[5]]
    s_list = []
    s_list.append(s)
    while t <= t_f:
        arr = f(s, t, t_to_v(t, feature), t_to_omega_z(t))
        # print(arr[0,0]*h)
        s = [s[0] + arr[0,0]*h, s[1] + arr[1,0]*h, s[2] + 0,
             s[3] + 0,          s[4] + 0,          normalizedHeadingRad(s[5] + arr[2,0]*h)]
        s_list.append(s)
        t = t + h
    # print(s_list)

    predicted_path = Path()
    predicted_path.header.frame_id = 'path'
    for s in s_list:
        pose = PoseStamped()
        pose.pose.position.x = s[0]
        pose.pose.position.y = s[1]
        pose.pose.orientation.w = 1.0
        predicted_path.poses.append(pose)
    return predicted_path
    # pub.publish(predicted_path)
    # pl.show()



rospy.init_node('motion_prediction', anonymous=True)
rospy.Subscriber('/path_feature', Float64MultiArray, callback)
pub = rospy.Publisher('/predicted_path', Path, queue_size=10)
rospy.spin()
