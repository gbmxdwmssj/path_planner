#!/usr/bin/env python

import rospy
import math
from scipy.interpolate import interp1d
from nav_msgs.msg import Path

v_0 = 0.0 # m/s
a_0 = 2.0 # m/s^2
v_tra = 3.0
a_f = -2.0
v_f = 0.0
t_f = None

def callback(path):
    if (len(path.poses) == 0):
        return
    print('start process!!!!!!')
    print(len(path.poses))

    ## compute the lenght of the path
    l_f = 0.0
    l_list = []
    theta_z_list = []
    # for i in range(len(path.poses) - 1):
    for i in range(len(path.poses) - 1, 0, -1):
        l_list.append(l_f)
        euler = tf.transformations.euler_from_quaternion(path.poses[i].pose.orientation)
        theta_z_list.append(euler[2])

        x = path.poses[i].pose.position.x
        y = path.poses[i].pose.position.y
        dx = x - path.poses[i-1].pose.position.x
        dy = y - path.poses[i-1].pose.position.y
        l_f = l_f + math.sqrt(dx*dx + dy*dy)
    print('path length')
    print(l_f)

    frac_1 = (v_tra - v_0)*(v_tra - v_0) / (2 * a_0)
    frac_2 = (v_tra - v_f)*(v_tra - v_f) / (2 * a_f)
    t_f = (l_f + frac_1 - frac_2) / v_tra
    t_1 = (v_tra - v_0) / a_0
    t_2 = t_f - ((v_f - v_tra) / a_f)
    print('t_f')
    print(t_f)
    print(t_1)
    print(t_2)

    K = 9
    Delta_t = t_f / (K - 1)
    trape_1_area = (v_0 + v_tra) * t_1 * 0.5
    for k in range(K):
        t = k * Delta_t
        if (t <= t_1):
            l = (v_0 + (v_0 + a_0 * t)) * t * 0.5
        elif (t <= t_2):
            l = trape_1_area + v_tra * (t - t_1)
        else:
            up = v_tra + a_f * (t - t_2)
            l = trape_1_area + v_tra * (t_2 - t_1) + (up + v_tra) * (t - t_2) * 0.5
        print('l')
        print(l)

rospy.init_node('feature_extraction', anonymous=True)
rospy.Subscriber('/sPath', Path, callback)
rospy.spin()

