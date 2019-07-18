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
from geometry_msgs.msg import *
from hybrid_astar.srv import *
from grid_map_msgs.msg import *
import time
import yaml
from matplotlib.pyplot import MultipleLocator
import os



s_path = None
o_path = None
p_path = None
ele_map = None
fil_map = None
start = None
goal = None
sgn = lambda x: 1 if x > 0 else -1 if x < 0 else 0

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



def startCallback(data):
    global start
    start = data



def goalCallback(data):
    global goal
    goal = data



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
        dx = path[i-1].pose.position.x - x
        dy = path[i-1].pose.position.y - y
        dis = math.sqrt(dx*dx + dy*dy)
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



rospy.init_node('performance', anonymous=True)
rospy.Subscriber('/sPath', Path, sCallback)
rospy.Subscriber('/oPath', Path, oCallback)
rospy.Subscriber('/predicted_path', Path, pCallback)
rospy.Subscriber('/grid_map_visualization/elevation_grid', OccupancyGrid, eleCallback)
rospy.Subscriber('/grid_map_filter_demo/filtered_map', GridMap, filCallback)
rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, startCallback)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalCallback)
start_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10, latch=True)
# print('I waiting for service /euler_from_quaternion...')
# rospy.wait_for_service('/euler_from_quaternion')
# print('Service /euler_from_quaternion is availiable!')

major_locator=MultipleLocator(0.25)



init_ele = None
# euler_from_quaternion = rospy.ServiceProxy('/euler_from_quaternion', EulerFromQuaternion)
while not rospy.core.is_shutdown():

    print('Missing for start or goal...')
    while (start is None or goal is None) and not rospy.core.is_shutdown():
        time.sleep(0.5)

    if not rospy.core.is_shutdown():
        print('Launch!')
        xs = []
        ys = []
        eles = []
        ls = []
        l = 0.0
        rolls = []
        pitchs = []
        yaws = []
        W = 1.7
        L = 2.0
        smooth_cost = 0.0

        rospy.set_param('/hybrid_astar/s_path_name', '/NoTS_sPath')
        rospy.set_param('/hybrid_astar/occ_thre', 45)
        rospy.set_param('/hybrid_astar/cost_mode', 0)
        rospy.set_param('/hybrid_astar/occ_wei', 0.0)
        start_pub.publish(start)
        goal_pub.publish(goal)
        print('---------- NoTS start! ----------')
        os.system('rosrun hybrid_astar hybrid_astar')
        print('NoTS stop!')
        time.sleep(1.0)

        rospy.set_param('/hybrid_astar/s_path_name', '/TS_NoSus_sPath')
        rospy.set_param('/hybrid_astar/occ_thre', 25)
        rospy.set_param('/hybrid_astar/cost_mode', 1)
        rospy.set_param('/hybrid_astar/occ_wei', 18.0)
        print('---------- TS_NoSus start! ----------')
        os.system('rosrun hybrid_astar hybrid_astar')
        print('TS_NoSus stop!')
        time.sleep(1.0)

        rospy.set_param('/hybrid_astar/s_path_name', '/TS_Sus_sPath')
        rospy.set_param('/hybrid_astar/occ_thre', 38)
        rospy.set_param('/hybrid_astar/cost_mode', 2)
        rospy.set_param('/hybrid_astar/occ_wei', 1.1)
        print('---------- TS_Sus start! ----------')
        os.system('rosrun hybrid_astar hybrid_astar')
        print('TS_Sus stop!')
        time.sleep(1.0)

        start = None
        goal = None

        # with open('/home/kai/performance_sheet.csv', 'w', newline='') as t_file:
        #     csv_writer = csv.writer(t_file)
        #     csv_writer.writerow(['length', l])
        #     csv_writer.writerow(['smooth_cost', smooth_cost])
        #     csv_writer.writerow(['t_f', t_f])

    else:
        print('Shutdown!')

else:
    print('Shutdown!')
