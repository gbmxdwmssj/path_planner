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



rospy.init_node('roll_pitch_plot', anonymous=True)
rospy.Subscriber('/sPath', Path, sCallback)
rospy.Subscriber('/oPath', Path, oCallback)
rospy.Subscriber('/predicted_path', Path, pCallback)

rospy.Subscriber('/NoTS_oPath', Path, NoTSCallback)
rospy.Subscriber('/TS_NoSus_oPath', Path, TS_NoSus_Callback)
rospy.Subscriber('/TS_Sus_oPath', Path, TS_Sus_Callback)

rospy.Subscriber('/grid_map_visualization/elevation_grid', OccupancyGrid, eleCallback)
rospy.Subscriber('/grid_map_filter_demo/filtered_map', GridMap, filCallback)
print('I waiting for service /euler_from_quaternion...')
rospy.wait_for_service('/euler_from_quaternion')
print('Service /euler_from_quaternion is availiable!')

major_locator=MultipleLocator(0.25)

# while (s_path is None or o_path is None or p_path is None or ele_map is None or fil_map is None) and not rospy.core.is_shutdown():
#     print('Wait for paths and map!')
#     time.sleep(0.5)



print('Missing the path of horizontal comparison...')
while (NoTS_path is None or TS_NoSus_path is None or TS_Sus_path is None or ele_map is None or fil_map is None) and not rospy.core.is_shutdown():
    time.sleep(0.5)



init_ele = None
euler_from_quaternion = rospy.ServiceProxy('/euler_from_quaternion', EulerFromQuaternion)
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

    # path = s_path

    # path = o_path
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
        euler = euler_from_quaternion(path.poses[i].pose.orientation)
        euler.yaw -= 0.25 * math.pi
        euler.yaw = -euler.yaw + 0.5 * math.pi
        yaw = normalizedHeadingRad(euler.yaw)
        yaws.append(yaw)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
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
        ##################################################
        yaw_mat = np.matrix(np.array([[ cy, sy],
                                      [-sy, cy]
                                     ]))
        wl_mat = np.matrix(np.array([[0.5*W],
                                     [L]
                                    ]))
        pt_rf = yaw_mat * wl_mat + np.matrix(np.array([[x],
                                                       [y]
                                                      ]))
        pt_rf = (np.array(pt_rf.T).tolist())[0]
        xtmp = pt_rf[0]
        ytmp = pt_rf[1]
        # xtmp *= 0.02
        # ytmp *= 0.02
        # xtmp = int(xtmp / reso)
        # ytmp = int(ytmp / reso)
        # ele = fil_map.data[2].data[(h-ytmp)*w+(w-xtmp)]
        # ele = (ele - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        # ele = ele_meter_range[0] + ele * (ele_meter_range[1] - ele_meter_range[0])
        ele = getEle(fil_map, 2, xtmp, ytmp)
        pt_rf += [ele]
        ##################################################
        yaw_mat = np.matrix(np.array([[-cy, sy],
                                      [ sy, cy]
                                     ]))
        pt_lf = yaw_mat * wl_mat + np.matrix(np.array([[x],
                                                       [y]
                                                      ]))
        pt_lf = (np.array(pt_lf.T).tolist())[0]
        xtmp = pt_lf[0]
        ytmp = pt_lf[1]
        # xtmp *= 0.02
        # ytmp *= 0.02
        # xtmp = int(xtmp / reso)
        # ytmp = int(ytmp / reso)
        # ele = fil_map.data[2].data[(h-ytmp)*w+(w-xtmp)]
        # ele = (ele - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        # ele = ele_meter_range[0] + ele * (ele_meter_range[1] - ele_meter_range[0])
        ele = getEle(fil_map, 2, xtmp, ytmp)
        pt_lf += [ele]
        ##################################################
        yaw_mat = np.matrix(np.array([[-cy, 0],
                                      [ sy, 0]
                                     ]))
        pt_lb = yaw_mat * wl_mat + np.matrix(np.array([[x],
                                                       [y]
                                                      ]))
        pt_lb = (np.array(pt_lb.T).tolist())[0]
        xtmp = pt_lb[0]
        ytmp = pt_lb[1]
        # xtmp *= 0.02
        # ytmp *= 0.02
        # xtmp = int(xtmp / reso)
        # ytmp = int(ytmp / reso)
        # ele = fil_map.data[2].data[(h-ytmp)*w+(w-xtmp)]
        # ele = (ele - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        # ele = ele_meter_range[0] + ele * (ele_meter_range[1] - ele_meter_range[0])
        ele = getEle(fil_map, 2, xtmp, ytmp)
        pt_lb += [ele]
        ##################################################
        yaw_mat = np.matrix(np.array([[ cy, 0],
                                      [-sy, 0]
                                     ]))
        pt_rb = yaw_mat * wl_mat + np.matrix(np.array([[x],
                                                       [y]
                                                      ]))
        pt_rb = (np.array(pt_rb.T).tolist())[0]
        xtmp = pt_rb[0]
        ytmp = pt_rb[1]
        # xtmp *= 0.02
        # ytmp *= 0.02
        # xtmp = int(xtmp / reso)
        # ytmp = int(ytmp / reso)
        # ele = fil_map.data[2].data[(h-ytmp)*w+(w-xtmp)]
        # ele = (ele - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])
        # ele = ele_meter_range[0] + ele * (ele_meter_range[1] - ele_meter_range[0])
        ele = getEle(fil_map, 2, xtmp, ytmp)
        pt_rb += [ele]

        roll1, pitch1 = getRollPitch(pt_rf, pt_lf, pt_lb)
        roll2, pitch2 = getRollPitch(pt_rf, pt_lf, pt_rb)
        roll3, pitch3 = getRollPitch(pt_rf, pt_lb, pt_rb)
        roll4, pitch4 = getRollPitch(pt_lf, pt_lb, pt_rb)
        roll  = max([abs(roll1), abs(roll2), abs(roll3), abs(roll4)])
        pitch = max([abs(pitch1), abs(pitch2), abs(pitch3), abs(pitch4)])
        roll_id  = np.argmax([abs(roll1), abs(roll2), abs(roll3), abs(roll4)])
        pitch_id = np.argmax([abs(pitch1), abs(pitch2), abs(pitch3), abs(pitch4)])
        tmp_rolls = [roll1, roll2, roll3, roll4]
        tmp_pitchs = [pitch1, pitch2, pitch3, pitch4]
        roll = sgn(tmp_rolls[roll_id]) * roll
        pitch = sgn(tmp_pitchs[pitch_id]) * pitch

        rolls.append(roll)
        pitchs.append(pitch)

        ele = 0.0
        # if init_ele is None:
        #     init_ele = ele
        #     plt.figure()
        #     plt.plot(x, y, 'o', pt_rf[0], pt_rf[1], 'x', pt_lf[0], pt_lf[1], 'x'
        #                       , pt_lb[0], pt_lb[1], 'x', pt_rb[0], pt_rb[1], 'x')
        #     plt.show()

        x *= 0.02
        y *= 0.02
        x = int(x / reso)
        y = int(y / reso)
        ele = fil_map.data[2].data[(h-y)*w+(w-x)]
        ele = (ele - ele_map_value_range[0]) / (ele_map_value_range[1] - ele_map_value_range[0])

    with open('/home/kai/roll_pitch_sheet.csv', 'w', newline='') as t_file:
        csv_writer = csv.writer(t_file)
        csv_writer.writerow(['l', 'roll', 'pitch'])
        for i in range(len(ls)):
            csv_writer.writerow([ls[i], rolls[i], pitchs[i]])

    plt.figure()
    plt.plot(ls, rolls)
    plt.show()

    plt.figure()
    plt.plot(ls, pitchs)
    plt.show()

else:
    print('Shutdown!')
