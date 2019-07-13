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
    dims = []
    data_offset = 0
    for path in paths:
        fea = calcFeature(path)
        fea_list = fea_list + fea.data
        print(len(fea.data))
        data_offset += len(fea.data)
        single_dim = MultiArrayDimension()
        single_dim.stride = data_offset
        dims.append(single_dim)
    print('number of features:')
    print(len(fea_list))
    # print(fea_list)
    features = Float64MultiArray()
    features.data = fea_list
    features.layout.dim = dims
    pub.publish(features)



rospy.init_node('spline', anonymous=True)
# print('I waiting for service /euler_from_quaternion...')
# rospy.wait_for_service('/euler_from_quaternion')
# print('Service /euler_from_quaternion is availiable!')
rospy.Subscriber('/sPath', Path, callback)
pub = rospy.Publisher('/oPath', Path, queue_size=10)

x = [0, 8,  8,  4,  5,  9]
y = [0, 2, 10, 11, 18, 23]
tck, u = interpolate.splprep([x, y], s=0)
unew = np.arange(0, 1.01, 0.01)
out = interpolate.splev(unew, tck)
plt.figure()
plt.plot(x, y, 'x', out[0], out[1])
plt.show()
print(u)

print('I will spin!')
# rospy.spin()
