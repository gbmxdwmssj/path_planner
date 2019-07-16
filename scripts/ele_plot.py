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



def eleCallback(map):
    global ele_map
    ele_map = map



def filCallback(map):
    global fil_map
    fil_map = map



rospy.init_node('ele_plot', anonymous=True)
rospy.Subscriber('/sPath', Path, sCallback)
rospy.Subscriber('/oPath', Path, oCallback)
rospy.Subscriber('/predicted_path', Path, pCallback)
rospy.Subscriber('/grid_map_visualization/elevation_grid', OccupancyGrid, eleCallback)
rospy.Subscriber('/grid_map_filter_demo/filtered_map', GridMap, filCallback)

while (s_path is None or o_path is None or p_path is None or ele_map is None or fil_map is None) and not rospy.core.is_shutdown():
    print('Wait for paths and map!')
    time.sleep(0.5)

# if not rospy.core.is_shutdown():
#     print('Draw!')
#     print(fil_map.data[2])
#     xs = []
#     ys = []
#     w = ele_map.info.width
#     h = ele_map.info.height
#     reso = ele_map.info.resolution
#     eles = []
#     ls = []
#     l = 0.0
#     for i in range(len(s_path.poses)-1, -1, -1):
#         ls.append(l)
#         x = s_path.poses[i].pose.position.x
#         y = s_path.poses[i].pose.position.y
#         if i != 0:
#             dx = s_path.poses[i-1].pose.position.x - x
#             dy = s_path.poses[i-1].pose.position.y - y
#         else:
#             dx = 0
#             dy = 0
#         dis = math.sqrt(dx*dx + dy*dy)
#         l += dis
#         xs.append(x)
#         ys.append(y)
#         print([x,y])
#         x *= 0.02
#         y *= 0.02
#         x = int(x / reso)
#         y = int(y / reso)
#         ele = ele_map.data[y*w+x]
#         ele = (ele - ele_range_in_occ[0]) / (ele_range_in_occ[1] - ele_range_in_occ[0])
#         eles.append(ele)
#         print(ele)

#     print(ls)
#     plt.figure()
#     plt.plot(ls, eles)
#     plt.show()

# else:
#     print('Shutdown!')

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
    for i in range(len(s_path.poses)-1, -1, -1):
        ls.append(l)
        x = s_path.poses[i].pose.position.x
        y = s_path.poses[i].pose.position.y
        if i != 0:
            dx = s_path.poses[i-1].pose.position.x - x
            dy = s_path.poses[i-1].pose.position.y - y
        else:
            dx = 0
            dy = 0
        dis = math.sqrt(dx*dx + dy*dy)
        l += dis
        xs.append(x)
        ys.append(y)
        x *= 0.02
        y *= 0.02
        x = int(x / reso)
        y = int(y / reso)
        ele = fil_map.data[2].data[(h-y)*w+(h-x)]
        ele = (ele - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        ele = ele_meter_range[0] + ele * (ele_meter_range[1] - ele_meter_range[0])
        if init_ele is None:
            init_ele = ele
        # eles.append(ele - init_ele)
        eles.append(ele)

    with open('/home/kai/ele_sheet.csv', 'w', newline='') as t_file:
        csv_writer = csv.writer(t_file)
        csv_writer.writerow(['l', 'elevation'])
        for i in range(len(ls)):
            csv_writer.writerow([ls[i], eles[i]])

    plt.figure()
    plt.plot(ls, eles)
    plt.show()

else:
    print('Shutdown!')
