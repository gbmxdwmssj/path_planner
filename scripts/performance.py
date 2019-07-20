#!/usr/bin/env python

import rospy
import math
import csv
import numpy as np
import scipy
from scipy import interpolate
from scipy.interpolate import CubicHermiteSpline
from nav_msgs.msg import *
from hybrid_astar.srv import *
from std_msgs.msg import *
import pylab as pl
import numpy as np
import matplotlib.pyplot as plt
import random
from geometry_msgs.msg import PoseStamped
from hybrid_astar.srv import *
from grid_map_msgs.msg import *
import time
import yaml
from matplotlib.pyplot import MultipleLocator



s_path = None
o_path = None
p_path = None

NoTS_path = None
TS_NoSus_path = None
TS_Sus_path = None

ele_map = None
fil_map = None
sgn = lambda x: 1 if x > 0 else -1 if x < 0 else 0

# file = open('/home/kai/catkin_ws/src/grid_map/grid_map_demos/config/filters_demo.yaml', 'r', encoding="utf-8")
# file_data = file.read()
# file.close()
# data = yaml.load(file_data)
# params = data['grid_map_visualization']['grid_map_visualizations'][2]['params']

# ele_range_in_meter = [params['data_min'], params['data_max']]
# print(ele_range_in_meter)
# ele_range_in_occ = [100, 0]

ele_map_value_range = [-0.5, 1.0]
grayscale_range = [0, 254]
ele_meter_range = [0, 198] # m



def normalizedHeadingRad(theta_z): # [0, 2pi)
    if theta_z < 0:
        theta_z = theta_z - 2.0 * math.pi * (int)(theta_z / (2.0 * math.pi))
        return 2.0 * math.pi + theta_z

    return theta_z - 2.0 * math.pi * (int)(theta_z / (2.0 * math.pi))



def sCallback(path):
    global s_path
    s_path = path



def oCallback(path):
    global o_path
    o_path = path



def pCallback(path):
    global p_path
    p_path = path



def NoTSCallback(path):
    global NoTS_path
    NoTS_path = path



def TS_NoSus_Callback(path):
    global TS_NoSus_path
    TS_NoSus_path = path



def TS_Sus_Callback(path):
    global TS_Sus_path
    TS_Sus_path = path



def eleCallback(map):
    global ele_map
    ele_map = map



def filCallback(map):
    global fil_map
    fil_map = map



def getRollPitch(p1, p2, p3):
    a = 1.0 * (p2[1] - p1[1]) * (p3[2] - p1[2]) - (p3[1] - p1[1]) * (p2[2] - p1[2])
    b = 1.0 * (p2[2] - p1[2]) * (p3[0] - p1[0]) - (p3[2] - p1[2]) * (p2[0] - p1[0])
    c = 1.0 * (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1])
    roll  = sgn(b) * math.acos(c / math.sqrt(b*b+c*c))
    pitch = sgn(a) * math.acos(math.sqrt((b*b+c*c) / (a*a+b*b+c*c)))
    return roll, pitch



def getSingleTf(path):
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
    for i in range(len(path) - 1, 0, -1):
        l_list.append(l_f)
        x = path[i].pose.position.x
        y = path[i].pose.position.y
        x1 = path[i-1].pose.position.x
        y1 = path[i-1].pose.position.y
        dx = x1 - x
        dy = y1 - y

        z = getMapValue(fil_map, 2, x, y)
        z = (z - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        z = ele_meter_range[0] + z * (ele_meter_range[1] - ele_meter_range[0])

        z1 = getMapValue(fil_map, 2, x1, y1)
        z1 = (z1 - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        z1 = ele_meter_range[0] + z1 * (ele_meter_range[1] - ele_meter_range[0])

        dz = z1 - z

        dis = math.sqrt(dx*dx + dy*dy + dz*dz)
        if v_tra < 0:
            dis = -dis
        l_f = l_f + dis

    l_list.append(l_f)
    frac_1 = (v_tra - v_0)*(v_tra - v_0) / (2 * a_0)
    frac_2 = (v_tra - v_f)*(v_tra - v_f) / (2 * a_f)
    t_f = (l_f + frac_1 - frac_2) / v_tra
    print(l_f)
    return t_f



def getTf(path):
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

    t_f = 0.0
    for path in paths:
        t_f += getSingleTf(path)

    return t_f



def getMapValue(in_map, layer, x, y):
    max_x = in_map.info.length_x
    max_y = in_map.info.length_y
    w = int(max_x / in_map.info.resolution + 0.5)
    h = int(max_y / in_map.info.resolution + 0.5)
    x *= 0.02
    y *= 0.02
    x = int(x / reso)
    y = int(y / reso)
    return in_map.data[layer].data[(h-y)*w+(w-x)]



rospy.init_node('performance', anonymous=True)
rospy.Subscriber('/sPath', Path, sCallback)
rospy.Subscriber('/oPath', Path, oCallback)
rospy.Subscriber('/predicted_path', Path, pCallback)

rospy.Subscriber('/NoTS_oPath', Path, NoTSCallback)
rospy.Subscriber('/TS_NoSus_oPath', Path, TS_NoSus_Callback)
rospy.Subscriber('/TS_Sus_oPath', Path, TS_Sus_Callback)

rospy.Subscriber('/grid_map_visualization/elevation_grid', OccupancyGrid, eleCallback)
rospy.Subscriber('/grid_map_filter_demo/filtered_map', GridMap, filCallback)
# print('I waiting for service /euler_from_quaternion...')
# rospy.wait_for_service('/euler_from_quaternion')
# print('Service /euler_from_quaternion is availiable!')

major_locator=MultipleLocator(0.25)

while (s_path is None or o_path is None or p_path is None or ele_map is None or fil_map is None) and not rospy.core.is_shutdown():
    print('Wait for paths and map!')
    time.sleep(0.5)



# print('Missing the path of horizontal comparison...')
# while (NoTS_path is None or TS_NoSus_path is None or TS_Sus_path is None or fil_map is None) and not rospy.core.is_shutdown():
#     time.sleep(0.5)



init_ele = None
# euler_from_quaternion = rospy.ServiceProxy('/euler_from_quaternion', EulerFromQuaternion)
if not rospy.core.is_shutdown():
    print('Draw!')
    xs = []
    ys = []
    # w = ele_map.info.width
    # h = ele_map.info.height
    # reso = ele_map.info.resolution
    eles = []
    ls = []
    l = 0.0
    rolls = []
    pitchs = []
    yaws = []
    W = 1.7
    L = 2.0
    smooth_cost = 0.0

    reso = fil_map.info.resolution

    # path = s_path

    # path = o_path
    # path.poses.reverse()

    path = p_path
    path.poses.reverse()

    # path = NoTS_path
    # path.poses.reverse()

    # path = TS_NoSus_path
    # path.poses.reverse()

    # path = TS_Sus_path
    # path.poses.reverse()

    v_cost = 0.35
    omega_z_cost = 0.1
    a_cost = 0.2
    alpha_z_cost = 0.03

    runtime = 2.790 # s

    # min_obs_dis = float('inf')
    # min_obs_dis = 15.4354
    # min_obs_dis = 18.0841
    min_obs_dis = 18.7232

    traversability = 0.0
    for i in range(len(path.poses)-2, 0, -1):
        ls.append(l)
        xim1 = path.poses[i+1].pose.position.x
        yim1 = path.poses[i+1].pose.position.y
        xi = path.poses[i].pose.position.x
        yi = path.poses[i].pose.position.y
        xip1 = path.poses[i-1].pose.position.x
        yip1 = path.poses[i-1].pose.position.y

        zim1 = getMapValue(fil_map, 2, xim1, yim1)
        zim1 = (zim1 - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        zim1 = ele_meter_range[0] + zim1 * (ele_meter_range[1] - ele_meter_range[0])

        zi = getMapValue(fil_map, 2, xi, yi)
        zi = (zi - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        zi = ele_meter_range[0] + zi * (ele_meter_range[1] - ele_meter_range[0])

        zip1 = getMapValue(fil_map, 2, xip1, yip1)
        zip1 = (zip1 - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        zip1 = ele_meter_range[0] + zip1 * (ele_meter_range[1] - ele_meter_range[0])

        vec_x = xip1 - 2.0 * xi + xim1
        vec_y = yip1 - 2.0 * yi + yim1
        vec_z = zip1 - 2.0 * zi + zim1
        smooth_cost += vec_x*vec_x + vec_y*vec_y + vec_z*vec_z
        if i != 0:
            dx = xip1 - xi
            dy = yip1 - yi
            dz = zip1 - zi
        else:
            dx = 0
            dy = 0
            dz = 0
        dis = math.sqrt(dx*dx + dy*dy + dz*dz)
        l += dis

        traversability += getMapValue(fil_map, 10, xi, yi)

    xi = path.poses[-1].pose.position.x
    yi = path.poses[-1].pose.position.y
    xip1 = path.poses[-2].pose.position.x
    yip1 = path.poses[-2].pose.position.y
    zi = getMapValue(fil_map, 2, xi, yi)
    zi = (zi - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
    zi = ele_meter_range[0] + zi * (ele_meter_range[1] - ele_meter_range[0])
    zip1 = getMapValue(fil_map, 2, xip1, yip1)
    zip1 = (zip1 - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
    zip1 = ele_meter_range[0] + zip1 * (ele_meter_range[1] - ele_meter_range[0])
    dx = xip1 - xi
    dy = yip1 - yi
    dz = zip1 - zi
    l += math.sqrt(dx*dx + dy*dy + dz*dz)
    print('l:', l)

    traversability += getMapValue(fil_map, 10, xi, yi)
    xi = path.poses[0].pose.position.x
    yi = path.poses[0].pose.position.y
    traversability += getMapValue(fil_map, 10, xi, yi)
    traversability /= len(path.poses)

    smooth_cost /= len(path.poses)-2
    print('smooth_cost:', smooth_cost)

    t_f = getTf(path)
    print('t_f:', t_f)

    print('traversability:', traversability)

    l_cost = l / 500.0
    nor_smooth_cost = smooth_cost / 0.01
    obs_cost = 1.0 / min_obs_dis
    t_f_cost = t_f / 150.0
    traversability_cost = 1.0 - traversability

    group_1_cost = nor_smooth_cost
    group_2_cost = l_cost + t_f_cost
    group_3_cost = v_cost + omega_z_cost + a_cost + alpha_z_cost
    group_4_cost = obs_cost + traversability_cost
    total_cost = (group_1_cost + group_2_cost + group_3_cost + group_4_cost) / 9.0

    with open('/home/kai/performance_sheet.csv', 'w', newline='') as t_file:
        csv_writer = csv.writer(t_file)
        csv_writer.writerow(['length', l])
        csv_writer.writerow(['smooth_cost', smooth_cost])
        csv_writer.writerow(['min_obs_dis', min_obs_dis])
        csv_writer.writerow(['t_f', t_f])
        csv_writer.writerow(['runtime', runtime])
        csv_writer.writerow(['traversability', traversability])
        csv_writer.writerow(['v_cost', v_cost])
        csv_writer.writerow(['omega_z_cost', omega_z_cost])
        csv_writer.writerow(['a_cost', a_cost])
        csv_writer.writerow(['alpha_z_cost', alpha_z_cost])
        csv_writer.writerow(['total_cost', total_cost])

else:
    print('Shutdown!')
