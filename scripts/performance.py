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



rospy.init_node('performance', anonymous=True)
rospy.Subscriber('/sPath', Path, sCallback)
rospy.Subscriber('/oPath', Path, oCallback)
rospy.Subscriber('/predicted_path', Path, pCallback)
rospy.Subscriber('/grid_map_visualization/elevation_grid', OccupancyGrid, eleCallback)
rospy.Subscriber('/grid_map_filter_demo/filtered_map', GridMap, filCallback)
# print('I waiting for service /euler_from_quaternion...')
# rospy.wait_for_service('/euler_from_quaternion')
# print('Service /euler_from_quaternion is availiable!')

major_locator=MultipleLocator(0.25)

while (s_path is None or o_path is None or p_path is None or ele_map is None or fil_map is None) and not rospy.core.is_shutdown():
    print('Wait for paths and map!')
    time.sleep(0.5)



init_ele = None
# euler_from_quaternion = rospy.ServiceProxy('/euler_from_quaternion', EulerFromQuaternion)
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
    rolls = []
    pitchs = []
    yaws = []
    W = 1.7
    L = 2.0
    smooth_cost = 0.0
    path = o_path
    for i in range(len(path.poses)-2, 0, -1):
        ls.append(l)
        xim1 = path.poses[i-1].pose.position.x
        yim1 = path.poses[i-1].pose.position.y
        xi = path.poses[i].pose.position.x
        yi = path.poses[i].pose.position.y
        xip1 = path.poses[i+1].pose.position.x
        yip1 = path.poses[i+1].pose.position.y
        vec_x = xip1 - 2.0 * xi + xim1
        vec_y = yip1 - 2.0 * yi + yim1
        smooth_cost += vec_x*vec_x + vec_y*vec_y
        if i != 0:
            dx = xim1 - xi
            dy = yim1 - yi
        else:
            dx = 0
            dy = 0
        dis = math.sqrt(dx*dx + dy*dy)
        l += dis

    smooth_cost /= len(path.poses)-2
    print(smooth_cost)

    # with open('/home/kai/roll_pitch_sheet.csv', 'w', newline='') as t_file:
    #     csv_writer = csv.writer(t_file)
    #     csv_writer.writerow(['l', 'roll', 'pitch'])
    #     for i in range(len(ls)):
    #         csv_writer.writerow([ls[i], rolls[i], pitchs[i]])

    # plt.figure()
    # plt.plot(ls, rolls)
    # plt.show()

    # plt.figure()
    # plt.plot(ls, pitchs)
    # plt.show()

else:
    print('Shutdown!')
