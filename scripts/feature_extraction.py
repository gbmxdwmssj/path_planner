#!/usr/bin/env python

import rospy
import math
from scipy.interpolate import interp1d
from nav_msgs.msg import Path
from hybrid_astar.srv import *

v_0 = 0.0 # m/s
a_0 = 2.0 # m/s^2
v_tra = 3.0
a_f = -2.0
v_f = 0.0
t_f = None

def normalizedHeadingRad(theta_z):
    if theta_z < 0:
        theta_z = theta_z - 2.0 * math.pi * (int)(theta_z / (2.0 * math.pi))
        return 2.0 * math.pi + theta_z

    return theta_z - 2.0 * math.pi * (int)(theta_z / (2.0 * math.pi))

def callback(path):
    if len(path.poses) == 0:
        return
    print('start process!!!!!!')
    print(len(path.poses))

    ## compute the lenght of the path
    l_f = 0.0
    l_list = []
    theta_z_list = []
    euler_from_quaternion = rospy.ServiceProxy('/euler_from_quaternion', EulerFromQuaternion)
    # for i in range(len(path.poses) - 1):
    for i in range(len(path.poses) - 1, 0, -1):
        l_list.append(l_f)
        
        euler = euler_from_quaternion(path.poses[i].pose.orientation)
        yaw = normalizedHeadingRad(euler.yaw)
        theta_z_list.append(yaw)
        print(yaw)

        x = path.poses[i].pose.position.x
        y = path.poses[i].pose.position.y
        dx = x - path.poses[i-1].pose.position.x
        dy = y - path.poses[i-1].pose.position.y
        l_f = l_f + math.sqrt(dx*dx + dy*dy)
    l_list.append(l_f)
    euler = euler_from_quaternion(path.poses[0].pose.orientation)
    yaw = normalizedHeadingRad(euler.yaw)
    theta_z_list.append(yaw)
    print('path length')
    print(l_f)
    print('l_list')
    print(l_list)
    print('theta_z_list')
    print(theta_z_list)
    l_to_theta_z = interp1d(l_list, theta_z_list, kind='slinear')

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
        if t <= t_1:
            l = (v_0 + (v_0 + a_0 * t)) * t * 0.5
        elif t <= t_2:
            l = trape_1_area + v_tra * (t - t_1)
        else:
            up = v_tra + a_f * (t - t_2)
            l = trape_1_area + v_tra * (t_2 - t_1) + (up + v_tra) * (t - t_2) * 0.5
        print('l')
        print(l)
        if k == K - 1:
            l = l - 0.000001
        theta_z = l_to_theta_z(l)
        print('theta_z')
        print(theta_z)

rospy.init_node('feature_extraction', anonymous=True)
print('I waiting for service /euler_from_quaternion...')
rospy.wait_for_service('/euler_from_quaternion')
print('Service /euler_from_quaternion is availiable!')
rospy.Subscriber('/sPath', Path, callback)
rospy.spin()

