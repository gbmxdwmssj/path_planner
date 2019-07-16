#!/usr/bin/env python

import rospy
import math
import numpy as np
import scipy
from scipy import interpolate
from scipy.interpolate import CubicHermiteSpline
from nav_msgs.msg import Path
from hybrid_astar.srv import *
from std_msgs.msg import *
import pylab as pl
import numpy as np
import matplotlib.pyplot as plt
import random
from geometry_msgs.msg import PoseStamped
from hybrid_astar.srv import *



def getOrien(pt1, pt2):
    dx = pt2.x - pt1.x
    dy = pt2.y - pt1.y
    yaw = math.atan2(dy, dx)
    quaternion_from_euler = rospy.ServiceProxy('/quaternion_from_euler', QuaternionFromEuler)
    return quaternion_from_euler(0, 0, yaw).quaternion



def updateYaw(path):
    for i in range(len(path.poses) - 1):
        if path.poses[i].header.frame_id == 'forward':
            path.poses[i].pose.orientation = getOrien(path.poses[i].pose.position, path.poses[i+1].pose.position)
        else:
            path.poses[i].pose.orientation = getOrien(path.poses[i+1].pose.position, path.poses[i].pose.position)

    path.poses[len(path.poses)-1].pose.orientation = path.poses[len(path.poses)-2].pose.orientation
    return path



def getDisReso(path):
    x = path[0].pose.position.x
    y = path[0].pose.position.y
    dx = x - path[1].pose.position.x
    dy = y - path[1].pose.position.y
    dis = math.sqrt(dx*dx + dy*dy)
    return dis



def getLength(path):
    l_f = 0.0
    for i in range(len(path) - 1):
        x = path[i].pose.position.x
        y = path[i].pose.position.y
        dx = x - path[i+1].pose.position.x
        dy = y - path[i+1].pose.position.y
        dis = math.sqrt(dx*dx + dy*dy)
        l_f = l_f + dis
    return l_f



def getKnots(path):
    x_list = []
    y_list = []
    min_knot_num = 4
    min_dis_step = 5.0 # m
    # max_dis_step = getLength(path) / (min_knot_num - 1) # m
    max_dis_step = 40.0 # m
    min_idx_step = int(min_dis_step / getDisReso(path))
    max_idx_step = int(max_dis_step / getDisReso(path))
    min_idx_step = max(1, min_idx_step)
    max_idx_step = max(1, max_idx_step)
    print(min_idx_step)
    print(max_idx_step)
    min_idx_step = min(min_idx_step, max_idx_step)
    idx = 0
    idx_list = []
    while idx < len(path):
        x_list.append(path[idx].pose.position.x)
        y_list.append(path[idx].pose.position.y)
        # rand_tmp = random.randint(min_idx_step, max_idx_step)
        rand_tmp = random.randint(min_idx_step, int(max_idx_step + (min_idx_step-max_idx_step)*(idx/len(path))))
        idx += rand_tmp
    didx = (len(path) - 1) - (idx - rand_tmp)
    if didx < min_idx_step:
        x_list[len(x_list)-1] = path[len(path)-1].pose.position.x
        y_list[len(y_list)-1] = path[len(path)-1].pose.position.y
    else:
        x_list.append(path[len(path)-1].pose.position.x)
        y_list.append(path[len(path)-1].pose.position.y)

    return [x_list, y_list]



def getKnotsDeris(path):
    knots = []
    deris = []
    u_list = []
    min_knot_num = 4
    min_dis_step = 5.0 # m
    max_dis_step = getLength(path) / (min_knot_num - 1) # m
    min_idx_step = int(min_dis_step / getDisReso(path))
    max_idx_step = int(max_dis_step / getDisReso(path))
    min_idx_step = max(1, min_idx_step)
    max_idx_step = max(1, max_idx_step)
    print(min_idx_step)
    print(max_idx_step)
    min_idx_step = min(min_idx_step, max_idx_step)
    idx = 0
    idx_list = []
    while idx < len(path):
        u_list.append(1.0 * idx / (len(path) - 1))
        x = path[idx].pose.position.x
        y = path[idx].pose.position.y
        knots.append([x,y])
        if idx < len(path) - 1:
            dx = path[idx+1].pose.position.x - x
            dy = path[idx+1].pose.position.y - y
        else:
            dx = x - path[idx-1].pose.position.x
            dy = y - path[idx-1].pose.position.y
        deris.append([dx,dy])
        rand_tmp = random.randint(min_idx_step, max_idx_step)
        idx += rand_tmp
    didx = (len(path) - 1) - (idx - rand_tmp)
    x = path[-1].pose.position.x
    y = path[-1].pose.position.y
    if didx < min_idx_step:
        knots[-1] = [x,y]
        dx = (x - path[-2].pose.position.x) * 100
        dy = (y - path[-2].pose.position.y) * 100
        deris[-1] = [dx,dy]
        u_list[-1] = 1.0
    else:
        knots.append([x,y])
        dx = (x - path[-2].pose.position.x) * 100
        dy = (y - path[-2].pose.position.y) * 100
        deris.append([dx,dy])
        u_list.append(1.0)

    return u_list, knots, deris



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

    fea_list = []
    dims = []
    data_offset = 0
    whole_path = Path()
    whole_path.header.frame_id = 'path'
    for path in paths:
        cur_dir = path[0].header.frame_id
        knots = getKnots(path)
        m = len(knots[0])
        if m <= 1:
            continue
        tck, u = interpolate.splprep(knots, k=min(3, m-1))
        size = int(getLength(path) / getDisReso(path) + 0.5)
        # print('l_f_l_f_l_f_l_spline_f_l_f_l_f_l_f')
        # print(getLength(path))
        unew = np.arange(0, 1.0 + 1.0/size, 1.0/size)
        out = interpolate.splev(unew, tck)
        # print(out)

        optim_path = Path()
        optim_path.header.frame_id = 'path'
        for i in range(len(out[0])):
            pose = PoseStamped()
            pose.pose.position.x = out[0][i]
            pose.pose.position.y = out[1][i]
            pose.header.frame_id = cur_dir
            optim_path.poses.append(pose)
        optim_path = updateYaw(optim_path)
        whole_path.poses += optim_path.poses

    pub.publish(whole_path)



# def callback(path):
#     if len(path.poses) == 0:
#         return

#     ## division
#     single_path = []
#     paths = []
#     for i in range(len(path.poses) - 1, 0, -1):
#         single_path.append(path.poses[i])
#         cur_dir = path.poses[i].header.frame_id
#         next_dir = path.poses[i-1].header.frame_id
#         if cur_dir != next_dir:
#             if len(single_path) >= 2:
#                 paths.append(single_path)
#             single_path = []
#     single_path.append(path.poses[0])
#     if len(single_path) >= 2:
#         paths.append(single_path)

#     fea_list = []
#     dims = []
#     data_offset = 0
#     whole_path = Path()
#     whole_path.header.frame_id = 'path'
#     for path in paths:
#         cur_dir = path[0].header.frame_id
#         u_list, knots, deris = getKnotsDeris(path)
#         m = len(knots[0])
#         if m <= 1:
#             continue
#         spline_poly = CubicHermiteSpline(u_list, knots, deris)
#         size = int(getLength(path) / getDisReso(path) + 0.5)
#         # print('l_f_l_f_l_f_l_spline_f_l_f_l_f_l_f')
#         # print(getLength(path))
#         unew = np.arange(0, 1.0 + 1.0/size, 1.0/size)
#         out = spline_poly(unew)
#         # print(out)

#         optim_path = Path()
#         optim_path.header.frame_id = 'path'
#         for i in range(len(out)):
#             pose = PoseStamped()
#             pose.pose.position.x = out[i][0]
#             pose.pose.position.y = out[i][1]
#             pose.header.frame_id = cur_dir
#             optim_path.poses.append(pose)
#         optim_path = updateYaw(optim_path)
#         whole_path.poses += optim_path.poses

#     pub.publish(whole_path)
        



rospy.init_node('spline', anonymous=True)
rospy.Subscriber('/sPath', Path, callback)
pub = rospy.Publisher('/oPath', Path, queue_size=10)
# rospy.wait_for_service('/quaternion_from_euler')

# reso = 0.2
# u = np.arange(0, 1.0 + reso, reso)
# knots = [[0,0],
#          [1,1],
#          [2,1],
#          [2,2],
#          [3,2],
#          [3,3]]
# deris = [[100,0],
#          [0,100],
#          [0,0],
#          [0,0],
#          [0,0],
#          [0,0]]
# spline_poly = CubicHermiteSpline(u, knots, deris)
# reso = 0.02
# unew = np.arange(0, 1.0 + reso, reso)
# out = spline_poly(unew)
# plt.figure()
# plt.plot(np.take(knots, 0, axis=1), np.take(knots, 1, axis=1), 'x', np.take(out, 0, axis=1), np.take(out, 1, axis=1))
# plt.show()

print('I will spin!')
rospy.spin()
