#!/usr/bin/env python

import rospy
import math
import numpy as np
import pylab as pl
from std_msgs.msg import Float64MultiArray
from scipy.interpolate import interp1d
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from hybrid_astar.srv import *
import random



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
    # v = v * (1.1 - 0.2 * random.random())
    # omega_z = omega_z * (1.5 - 1.0 * random.random())
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
    euler_from_quaternion = rospy.ServiceProxy('/euler_from_quaternion', EulerFromQuaternion)
    print('I get some features!')
    ## division
    dims = features.layout.dim
    fea_num = len(dims)
    fea_list = []
    for i in range(fea_num):
        if i == 0:
            fea_list.append(list(features.data[0:dims[i].stride]))
        else:
            fea_list.append(list(features.data[dims[i-1].stride:dims[i].stride]))
    # print(fea_list)
    whole_path = Path()
    whole_path.header.frame_id = 'path'
    path = None
    for fea in fea_list:
        feature = Float64MultiArray()
        feature.data = fea
        # if path is None:
        if True:
            path = calc_predicted_path(feature,
                feature.data[0], feature.data[1], feature.data[2],
                feature.data[3], feature.data[4], feature.data[5])
        else:
            tlen = len(path.poses)
            posi = path.poses[tlen-1].pose.position
            euler = euler_from_quaternion(path.poses[tlen-1].pose.orientation)
            path = calc_predicted_path(feature,
                posi.x, posi.y, posi.z,
                euler.roll, euler.pitch, euler.yaw)
        whole_path.poses += path.poses
    pub.publish(whole_path)
    # pl.show()



def calc_predicted_path(feature, x0, y0, z0, theta_x0, theta_y0, theta_z0):
    quaternion_from_euler = rospy.ServiceProxy('/quaternion_from_euler', QuaternionFromEuler)
    t_f = feature.data[11]
    t_list = []
    omega_z_list = []
    K = len(feature.data) - 12
    Delta_t = t_f / (K - 1)
    for k in range(K):
        t_list.append(k * Delta_t)

    for i in range(12, len(feature.data), 1):
        omega_z_list.append(feature.data[i])

    t_to_omega_z = interp1d(t_list, omega_z_list, kind='slinear')
    # t_new = np.linspace(0, t_f - 0.1, 1000)
    # omega_z_new = t_to_omega_z(t_new)
    # pl.plot(t_new, omega_z_new)

    h = rospy.get_param('/hybrid_astar/dt', 0.1) # s
    t = 0
    s = [x0, y0, z0, theta_x0, theta_y0, theta_z0]
    s_list = []
    s_list.append(s)
    while t <= t_f:
        arr = f(s, t, t_to_v(t, feature), t_to_omega_z(t))
        s = [s[0] + arr[0,0]*h, s[1] + arr[1,0]*h, s[2] + 0,
             s[3] + 0,          s[4] + 0,          normalizedHeadingRad(s[5] + arr[2,0]*h)]
        s_tmp = s.copy()
        # if random.random() < 0.2:
        #     s_tmp[0] = s_tmp[0] + (0.3 - 0.6 * random.random())
        #     s_tmp[1] = s_tmp[1] + (0.3 - 0.6 * random.random())
        #     s_tmp[5] = s_tmp[5] + (0.1 - 0.2 * random.random())
        s_list.append(s_tmp)
        t = t + h
    # print(s_list)

    predicted_path = Path()
    predicted_path.header.frame_id = 'path'
    for s in s_list:
        pose = PoseStamped()
        pose.pose.position.x = s[0]
        pose.pose.position.y = s[1]
        pose.pose.orientation = quaternion_from_euler(s[3], s[4], s[5]).quaternion
        predicted_path.poses.append(pose)
    return predicted_path



rospy.init_node('motion_prediction', anonymous=True)
rospy.Subscriber('/path_feature', Float64MultiArray, callback)
rospy.wait_for_service('/euler_from_quaternion')
rospy.wait_for_service('/quaternion_from_euler')
pub = rospy.Publisher('/predicted_path', Path, queue_size=10)
rospy.spin()
