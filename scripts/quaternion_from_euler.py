#!/usr/bin/env python2

import rospy
import tf
from hybrid_astar.srv import *
from geometry_msgs.msg import Quaternion

def quaternionFromEuler(req):
    q = tf.transformations.quaternion_from_euler(req.roll, req.pitch, req.yaw) # rad
    quaternion = Quaternion()
    quaternion.x = q[0]
    quaternion.y = q[1]
    quaternion.z = q[2]
    quaternion.w = q[3]
    return quaternion

rospy.init_node('quaternion_euler_server', anonymous=True)
s = rospy.Service('/quaternion_from_euler', QuaternionFromEuler, quaternionFromEuler)
print('Ready to calculate quaternion from euler.')
rospy.spin()

