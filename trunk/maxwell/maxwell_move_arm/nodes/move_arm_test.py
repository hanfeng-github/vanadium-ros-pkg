#!/usr/bin/env python

"""
  Test the move_arm.py program

  usage:
    move_arm_test.py x y z wrist_pitch [wrist_roll frame_id]
"""

import roslib; roslib.load_manifest('maxwell_move_arm')
import rospy
import sys

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from maxwell_move_arm.srv import * 

if __name__ == '__main__':
    rospy.init_node('move_arm_test')
    rospy.wait_for_service('move_arm/move')
    move_srv = rospy.ServiceProxy('move_arm/move', MoveArm) 

    req = MoveArmRequest()  
    req.pose_stamped.header.frame_id = "torso_link"
    if len(sys.argv) > 6:
        req.pose_stamped.header.frame_id = sys.argv[6]

    req.pose_stamped.pose.position.x = float(sys.argv[1])
    req.pose_stamped.pose.position.y = float(sys.argv[2])
    req.pose_stamped.pose.position.z = float(sys.argv[3])

    roll = 0.0
    if len(sys.argv) > 5:
        roll = float(sys.argv[5])
    q = quaternion_from_euler(roll, float(sys.argv[4]), 0.0, 'sxyz')
    req.pose_stamped.pose.orientation.x = q[0]
    req.pose_stamped.pose.orientation.y = q[1]
    req.pose_stamped.pose.orientation.z = q[2]
    req.pose_stamped.pose.orientation.w = q[3]

    try:
        r = move_srv(req)
        print r
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)

