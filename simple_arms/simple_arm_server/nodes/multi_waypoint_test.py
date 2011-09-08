#!/usr/bin/env python

"""
  simple_arm_server_test.py - tests the simple_arm_server.py program
  Copyright (c) 2011 Michael Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holders nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest('simple_arm_server')
import rospy
import sys

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from simple_arm_server.msg import *
from simple_arm_server.srv import * 

if __name__ == '__main__':
    rospy.init_node('simple_arm_server_test')
    rospy.wait_for_service('simple_arm_server/move')
    move_srv = rospy.ServiceProxy('simple_arm_server/move', MoveArm) 

    req = MoveArmRequest()  
    req.header.frame_id = "base_link"
 
    action = ArmAction()
    action.type = ArmAction.MOVE_ARM
    action.goal.position.x = 0.2
    action.goal.position.y = -0.09
    action.goal.position.z = .1
    q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
    action.goal.orientation.x = q[0]
    action.goal.orientation.y = q[1]
    action.goal.orientation.z = q[2]
    action.goal.orientation.w = q[3]
    action.move_time = rospy.Duration(5.0)
    req.goals.append(action)

    action = ArmAction()
    action.type = ArmAction.MOVE_GRIPPER
    action.command = 0.03
    action.move_time = rospy.Duration(1.0)
    req.goals.append(action)

    action = ArmAction()
    action.type = ArmAction.MOVE_ARM
    action.goal.position.x = 0.2
    action.goal.position.y = -0.09
    action.goal.position.z = .04
    q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
    action.goal.orientation.x = q[0]
    action.goal.orientation.y = q[1]
    action.goal.orientation.z = q[2]
    action.goal.orientation.w = q[3]
    action.move_time = rospy.Duration(1.0)
    req.goals.append(action)

    action = ArmAction()
    action.type = ArmAction.MOVE_GRIPPER
    action.command = 0.0254
    action.move_time = rospy.Duration(2.0)
    req.goals.append(action)

    action = ArmAction()
    action.type = ArmAction.MOVE_ARM
    action.goal.position.x = 0.2
    action.goal.position.y = -0.09
    action.goal.position.z = .1
    q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
    action.goal.orientation.x = q[0]
    action.goal.orientation.y = q[1]
    action.goal.orientation.z = q[2]
    action.goal.orientation.w = q[3]
    action.move_time = rospy.Duration(1.0)
    req.goals.append(action)

    action = ArmAction()
    action.type = ArmAction.MOVE_ARM
    action.goal.position.x = 0.2
    action.goal.position.y = 0.09
    action.goal.position.z = .1
    q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
    action.goal.orientation.x = q[0]
    action.goal.orientation.y = q[1]
    action.goal.orientation.z = q[2]
    action.goal.orientation.w = q[3]
    action.move_time = rospy.Duration(5.0)
    req.goals.append(action)
    
    action = ArmAction()
    action.type = ArmAction.MOVE_GRIPPER
    action.command = 0.03
    action.move_time = rospy.Duration(2.0)
    req.goals.append(action)

    action = ArmAction()
    action.type = ArmAction.MOVE_GRIPPER
    action.command = 0.0254
    action.move_time = rospy.Duration(1.0)
    req.goals.append(action)

    try:
        r = move_srv(req)
        print r
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)
