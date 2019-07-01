#!/usr/bin/env python

import rospy
import math
import numpy as np
from scipy.interpolate import interp1d
from nav_msgs.msg import Path
from hybrid_astar.srv import *
from std_msgs.msg import Float64MultiArray
import pylab as pl

v_0 = 0.0 # m/s
a_0 = 2.0 # m/s^2
v_tra = 3.0
a_f = -2.0
v_f = 0.0
t_f = None

def radMinus(rad_1, rad_2): # [0, 2pi)
    assert (rad_1 >=0 and rad_1 < 2 * math.pi)
    assert (rad_2 >=0 and rad_2 < 2 * math.pi)
    abs_diff_1 = abs(rad_1 - rad_2)
    abs_diff_2 = 2 * math.pi - abs_diff_1
    if rad_1 >= rad_2 and abs_diff_1 >= abs_diff_2:
        return -abs_diff_2
    elif rad_1 >= rad_2 and abs_diff_1 < abs_diff_2:
        return abs_diff_1
    elif rad_1 < rad_2 and abs_diff_1 < abs_diff_2:
        return -abs_diff_1
    elif rad_1 < rad_2 and abs_diff_1 >= abs_diff_2:
        return abs_diff_2
    else:
        assert False

def t_to_l(t, v_feature):
    v_0 = v_feature.data[0]
    a_0 = v_feature.data[1]
    v_tra = v_feature.data[2]
    a_f = v_feature.data[3]
    v_f = v_feature.data[4]
    t_f = v_feature.data[5]
    t_1 = (v_tra - v_0) / a_0
    t_2 = t_f - ((v_f - v_tra) / a_f)
    trape_1_area = (v_0 + v_tra) * t_1 * 0.5
    if t <= t_1:
        t = max(0, t)
        l = (v_0 + (v_0 + a_0 * t)) * t * 0.5
    elif t <= t_2:
        l = trape_1_area + v_tra * (t - t_1)
    else:
        t = min(t, t_f)
        up = v_tra + a_f * (t - t_2)
        l = trape_1_area + v_tra * (t_2 - t_1) + (up + v_tra) * (t - t_2) * 0.5
    return l

def normalizedHeadingRad(theta_z):
    if theta_z < 0:
        theta_z = theta_z - 2.0 * math.pi * (int)(theta_z / (2.0 * math.pi))
        return 2.0 * math.pi + theta_z

    return theta_z - 2.0 * math.pi * (int)(theta_z / (2.0 * math.pi))

def callback(path):
    if len(path.poses) == 0:
        return

    ## compute the lenght of the path
    l_f = 0.0
    l_list = []
    theta_z_list = []
    euler_from_quaternion = rospy.ServiceProxy('/euler_from_quaternion', EulerFromQuaternion)
    for i in range(len(path.poses) - 1, 0, -1):
        l_list.append(l_f)
        
        euler = euler_from_quaternion(path.poses[i].pose.orientation)
        yaw = normalizedHeadingRad(euler.yaw)
        theta_z_list.append(yaw)

        x = path.poses[i].pose.position.x
        y = path.poses[i].pose.position.y
        dx = x - path.poses[i-1].pose.position.x
        dy = y - path.poses[i-1].pose.position.y
        dis = math.sqrt(dx*dx + dy*dy)
        l_f = l_f + dis
    l_list.append(l_f)
    euler = euler_from_quaternion(path.poses[0].pose.orientation)
    yaw = normalizedHeadingRad(euler.yaw)
    theta_z_list.append(yaw)
    l_to_theta_z = interp1d(l_list, theta_z_list, kind='slinear')
    l_new = np.linspace(0, l_f, 1000)
    theta_z_new = l_to_theta_z(l_new)
    print('theta_z_list:')
    print(theta_z_list)
    pl.plot(l_new, theta_z_new)

    frac_1 = (v_tra - v_0)*(v_tra - v_0) / (2 * a_0)
    frac_2 = (v_tra - v_f)*(v_tra - v_f) / (2 * a_f)
    t_f = (l_f + frac_1 - frac_2) / v_tra
    t_1 = (v_tra - v_0) / a_0
    t_2 = t_f - ((v_f - v_tra) / a_f)

    v_feature = Float64MultiArray()
    v_feature.data = [v_0, a_0, v_tra, a_f, v_f, t_f]

    K = 9
    Delta_t = t_f / (K - 1)
    omega_z_feature = Float64MultiArray()
    dt = rospy.get_param('/hybrid_astar/dt') # s
    for k in range(K):
        t = k * Delta_t
        l = t_to_l(t, v_feature)
        theta_z = l_to_theta_z(l)
        if k == K - 1:
            l_minus = t_to_l(t - dt, v_feature)
            l = l - 0.000001
        else:
            l_plus = t_to_l(t + dt, v_feature)

        if k == K - 1:
            theta_z_minus = l_to_theta_z(l_minus)
            print(theta_z)
            print(theta_z_minus)
            omega_z = radMinus(theta_z, theta_z_minus) / dt
        else:
            theta_z_plus = l_to_theta_z(l_plus)
            print(theta_z_plus)
            print(theta_z)
            omega_z = radMinus(theta_z_plus, theta_z) / dt

        omega_z_feature.data.append(omega_z)

    print(omega_z_feature.data)
    feature = Float64MultiArray()
    tmp_len = len(path.poses)
    x0 = path.poses[tmp_len-1].pose.position.x
    y0 = path.poses[tmp_len-1].pose.position.y
    euler = euler_from_quaternion(path.poses[tmp_len-1].pose.orientation)
    yaw = normalizedHeadingRad(euler.yaw)
    theta_z0 = yaw
    feature.data = [x0, y0, 0, 0, 0, theta_z0] + v_feature.data + omega_z_feature.data
    pub.publish(feature)
    pl.show()

rospy.init_node('feature_extraction', anonymous=True)
print('I waiting for service /euler_from_quaternion...')
rospy.wait_for_service('/euler_from_quaternion')
print('Service /euler_from_quaternion is availiable!')
rospy.Subscriber('/sPath', Path, callback)
pub = rospy.Publisher('/path_feature', Float64MultiArray, queue_size=10)
rospy.spin()

