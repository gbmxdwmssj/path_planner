#!/usr/bin/env python2

import rospy
import tf
from hybrid_astar.srv import *

def eulerFromQuaternion(req):
    print('I get a quaternion!')
    quaternion = (req.quaternion.x,
        req.quaternion.y,
        req.quaternion.z,
        req.quaternion.w)
    euler = tf.transformations.euler_from_quaternion(quaternion) # rad
    print('I compute a euler!')
    return EulerFromQuaternionResponse(euler[0], euler[1], euler[2]) # rad

rospy.init_node('euler_quaternion_server', anonymous=True)
s = rospy.Service('/euler_from_quaternion', EulerFromQuaternion, eulerFromQuaternion)
print('Ready to calculate euler from quaternion.')
rospy.spin()

