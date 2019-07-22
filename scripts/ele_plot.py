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



s_path = None
o_path = None
p_path = None

NoTS_path = None
TS_NoSus_path = None
TS_Sus_path = None

ele_map = None
fil_map = None

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



def getEle(in_map, layer, x, y):
    z = getMapValue(fil_map, layer, x, y)
    z = (z - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
    z = ele_meter_range[0] + z * (ele_meter_range[1] - ele_meter_range[0])
    return z



rospy.init_node('ele_plot', anonymous=True)
rospy.Subscriber('/sPath', Path, sCallback)
rospy.Subscriber('/oPath', Path, oCallback)
rospy.Subscriber('/predicted_path', Path, pCallback)

rospy.Subscriber('/NoTS_oPath', Path, NoTSCallback)
rospy.Subscriber('/TS_NoSus_oPath', Path, TS_NoSus_Callback)
rospy.Subscriber('/TS_Sus_oPath', Path, TS_Sus_Callback)

rospy.Subscriber('/grid_map_visualization/elevation_grid', OccupancyGrid, eleCallback)
rospy.Subscriber('/grid_map_filter_demo/filtered_map', GridMap, filCallback)

# while (s_path is None or o_path is None or p_path is None or ele_map is None or fil_map is None) and not rospy.core.is_shutdown():
#     print('Wait for paths and map!')
    # time.sleep(0.5)


print('Missing the path of horizontal comparison...')
while (NoTS_path is None or TS_NoSus_path is None or TS_Sus_path is None or ele_map is None or fil_map is None) and not rospy.core.is_shutdown():
    time.sleep(0.5)



init_ele = None
if not rospy.core.is_shutdown():
    print('Draw!')
    print(fil_map.data[2].layout)
    print(min(fil_map.data[2].data))
    print(max(fil_map.data[2].data))
    print(fil_map.info)
    xs = []
    ys = []
    w = ele_map.info.width
    h = ele_map.info.height
    print(w)
    print(h)
    reso = ele_map.info.resolution
    eles = []
    ls = []
    l = 0.0

    # path = s_path

    # path = o_path
    # path.poses.reverse()

    # path = p_path
    # path.poses.reverse()

    # path = NoTS_path
    # path.poses.reverse()

    # path = TS_NoSus_path
    # path.poses.reverse()

    path = TS_Sus_path
    path.poses.reverse()

    for i in range(len(path.poses)-1, -1, -1):
        ls.append(l)
        x = path.poses[i].pose.position.x
        y = path.poses[i].pose.position.y
        z = getEle(fil_map, 2, x, y)
        if i != 0:
            x1 = path.poses[i-1].pose.position.x
            y1 = path.poses[i-1].pose.position.y
            z1 = getEle(fil_map, 2, x1, y1)
            dx = x1 - x
            dy = y1 - y
            dz = z1 - z
        else:
            dx = 0
            dy = 0
            dz = 0
        dis = math.sqrt(dx*dx + dy*dy + dz*dz)
        l += dis
        xs.append(x)
        ys.append(y)
        x *= 0.02
        y *= 0.02
        x = int(x / reso)
        y = int(y / reso)
        ele = fil_map.data[2].data[(h-y)*w+(w-x)]
        ele = (ele - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        ele = ele_meter_range[0] + ele * (ele_meter_range[1] - ele_meter_range[0])
        if init_ele is None:
            init_ele = ele
        eles.append(ele)

    with open('/home/kai/ori_ele_sheet.csv', 'w', newline='') as t_file:
        csv_writer = csv.writer(t_file)
        csv_writer.writerow(['l', 'elevation'])
        for i in range(len(ls)):
            csv_writer.writerow([ls[i], eles[i]])

    plt.figure()
    plt.plot(ls, eles)
    plt.show()

else:
    print('Shutdown!')
