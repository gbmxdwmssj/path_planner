#!/usr/bin/env python

import rospy
import math
import numpy as np
from scipy.interpolate import interp1d
from nav_msgs.msg import Path
from hybrid_astar.srv import *
from std_msgs.msg import Float64MultiArray
import pylab as pl

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

    ## division
    single_path = []
    paths = []
    for i in range(len(path.poses) - 1, 0, -1):
        single_path.append(path.poses[i])
        cur_dir = path.poses[i].header.frame_id
        next_dir = path.poses[i-1].header.frame_id
        if cur_dir != next_dir:
            if len(single_path) >= 2:
                paths.append(single_path)
            single_path = []
    single_path.append(path.poses[0])
    if len(single_path) >= 2:
        paths.append(single_path)
    # print('number of paths:')
    # print(len(paths))

    fea_list = []
    for path in paths:
        fea = calcFeature(path)
        fea_list = fea_list + fea.data
        print(len(fea.data))
        data_offset = len(fea.data)
    print('number of features:')
    print(len(fea_list))
    # print(fea_list)
    features = Float64MultiArray()
    features.data = fea_list
    features.layout.data_offset = data_offset
    pub.publish(features)

def calcFeature(path):
    if path[0].header.frame_id == 'forward':
        print('forward!')
        v_0 = 0.0 # m/s
        a_0 = 2.0 # m/s^2
        v_tra = 3.0
        a_f = -2.0
        v_f = 0.0
        t_f = None
    else:
        print('reverse!')
        v_0 = 0.0 # m/s
        a_0 = -2.0 # m/s^2
        v_tra = -3.0
        a_f = 2.0
        v_f = 0.0
        t_f = None

    ## compute the lenght of the path
    l_f = 0.0
    l_list = []
    c_theta_z_list = []
    s_theta_z_list = []
    euler_from_quaternion = rospy.ServiceProxy('/euler_from_quaternion', EulerFromQuaternion)
    for i in range(len(path) - 1):
        l_list.append(l_f)
        
        euler = euler_from_quaternion(path[i].pose.orientation)
        yaw = normalizedHeadingRad(euler.yaw)
        c_theta_z = math.cos(yaw)
        s_theta_z = math.sin(yaw)
        c_theta_z_list.append(c_theta_z)
        s_theta_z_list.append(s_theta_z)

        x = path[i].pose.position.x
        y = path[i].pose.position.y
        dx = x - path[i+1].pose.position.x
        dy = y - path[i+1].pose.position.y
        dis = math.sqrt(dx*dx + dy*dy)
        if v_tra < 0:
            dis = -dis
        l_f = l_f + dis
    l_list.append(l_f)
    euler = euler_from_quaternion(path[0].pose.orientation)
    yaw = normalizedHeadingRad(euler.yaw)
    c_theta_z = math.cos(yaw)
    s_theta_z = math.sin(yaw)
    c_theta_z_list.append(c_theta_z)
    s_theta_z_list.append(s_theta_z)
    l_to_c_theta_z = interp1d(l_list, c_theta_z_list, kind='slinear')
    l_to_s_theta_z = interp1d(l_list, s_theta_z_list, kind='slinear')
    # l_new = np.linspace(0, l_f, 1000)
    # c_theta_z_new = l_to_c_theta_z(l_new)
    # pl.plot(l_new, c_theta_z_new)
    # pl.show()

    frac_1 = (v_tra - v_0)*(v_tra - v_0) / (2 * a_0)
    frac_2 = (v_tra - v_f)*(v_tra - v_f) / (2 * a_f)
    t_f = (l_f + frac_1 - frac_2) / v_tra
    t_1 = (v_tra - v_0) / a_0
    t_2 = t_f - ((v_f - v_tra) / a_f)

    v_feature = Float64MultiArray()
    v_feature.data = [v_0, a_0, v_tra, a_f, v_f, t_f]

    K = 200
    Delta_t = t_f / (K - 1)
    omega_z_feature = Float64MultiArray()
    dt = rospy.get_param('/hybrid_astar/dt') # s
    if v_tra < 0:
        bias =  0.000001
    else:
        bias = -0.000001

    for k in range(K):
        t = k * Delta_t
        l = t_to_l(t, v_feature)
        if k == K - 1:
            l_minus = t_to_l(t - dt, v_feature)
            l = l + bias
        else:
            l_plus = t_to_l(t + dt, v_feature)
            if l_plus > l_f:
                l_plus = l_plus + bias
        c_theta_z = min(max(l_to_c_theta_z(l), -1), 1)
        s_theta_z =  min(max(l_to_s_theta_z(l), -1), 1)
        if s_theta_z >= 0:
            theta_z = normalizedHeadingRad(math.acos(c_theta_z))
        else:
            theta_z = 2 * math.pi - normalizedHeadingRad(math.acos(c_theta_z))

        if k == K - 1:
            c_theta_z_minus = min(max(l_to_c_theta_z(l_minus), -1), 1)
            s_theta_z_minus =  min(max(l_to_s_theta_z(l_minus), -1), 1)
            if s_theta_z_minus >= 0:
                theta_z_minus = normalizedHeadingRad(math.acos(c_theta_z_minus))
            else:
                theta_z_minus = 2 * math.pi - normalizedHeadingRad(math.acos(c_theta_z_minus))
            # print(theta_z)
            # print(theta_z_minus)
            omega_z = radMinus(theta_z, theta_z_minus) / dt
        else:
            c_theta_z_plus = min(max(l_to_c_theta_z(l_plus), -1), 1)
            s_theta_z_plus =  min(max(l_to_s_theta_z(l_plus), -1), 1)
            if s_theta_z_plus >= 0:
                theta_z_plus = normalizedHeadingRad(math.acos(c_theta_z_plus))
            else:
                theta_z_plus = 2 * math.pi - normalizedHeadingRad(math.acos(c_theta_z_plus))
            # print(theta_z_plus)
            # print(theta_z)
            omega_z = radMinus(theta_z_plus, theta_z) / dt

        omega_z_feature.data.append(omega_z)

    # print(omega_z_feature.data)
    feature = Float64MultiArray()
    x0 = path[0].pose.position.x
    y0 = path[0].pose.position.y
    euler = euler_from_quaternion(path[0].pose.orientation)
    yaw = normalizedHeadingRad(euler.yaw)
    theta_z0 = yaw
    feature.data = [x0, y0, 0, 0, 0, theta_z0] + v_feature.data + omega_z_feature.data
    return feature

rospy.init_node('feature_extraction', anonymous=True)
print('I waiting for service /euler_from_quaternion...')
rospy.wait_for_service('/euler_from_quaternion')
print('Service /euler_from_quaternion is availiable!')
rospy.Subscriber('/sPath', Path, callback)
pub = rospy.Publisher('/path_feature', Float64MultiArray, queue_size=10)
rospy.spin()

