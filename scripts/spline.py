#!/usr/bin/env python

import rospy
import math
import numpy as np
from scipy.interpolate import interp1d
from nav_msgs.msg import Path
from hybrid_astar.srv import *
from std_msgs.msg import *
import pylab as pl
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import random
from geometry_msgs.msg import PoseStamped



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
        x_list.append(path[idx].pose.position.x)
        y_list.append(path[idx].pose.position.y)
        rand_tmp = random.randint(min_idx_step, max_idx_step)
        idx += rand_tmp
    didx = (len(path) - 1) - (idx - rand_tmp)
    if didx < min_idx_step:
        x_list[len(x_list)-1] = path[len(path)-1].pose.position.x
        y_list[len(y_list)-1] = path[len(path)-1].pose.position.y
    else:
        x_list.append(path[len(path)-1].pose.position.x)
        y_list.append(path[len(path)-1].pose.position.y)

    return [x_list, y_list]



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
        knots = getKnots(path)
        m = len(knots[0])
        if m <= 1:
            continue
        tck, u = interpolate.splprep(knots, s=0, k=min(3, m-1))
        size = int(getLength(path) / getDisReso(path) + 0.5)
        unew = np.arange(0, 1.0 + 1.0/size, 1.0/size)
        out = interpolate.splev(unew, tck)

        optim_path = Path()
        optim_path.header.frame_id = 'path'
        for i in range(len(out[0])):
            pose = PoseStamped()
            pose.pose.position.x = out[0][i]
            pose.pose.position.y = out[1][i]
            optim_path.poses.append(pose)
        whole_path.poses += optim_path.poses

    pub.publish(whole_path)
        



rospy.init_node('spline', anonymous=True)
rospy.Subscriber('/sPath', Path, callback)
pub = rospy.Publisher('/oPath', Path, queue_size=10)
print('I will spin!')
rospy.spin()
