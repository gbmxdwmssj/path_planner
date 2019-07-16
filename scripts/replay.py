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
import rosbag



rospy.init_node('replay', anonymous=True)
pub = rospy.Publisher('/sPath', Path, queue_size=10, latch=True)
bag = rosbag.Bag('/home/kai/map-11-paths.bag')
for topic, msg, t in bag.read_messages(topics=['/sPath']):
    # print(msg)
    print('-----------------------------------')
    print(msg.header)
    msg.header.stamp.secs = 0
    msg.header.stamp.nsecs = 0
    print(msg.header)
    pub.publish(msg)

bag.close()
print('I will spin!')
