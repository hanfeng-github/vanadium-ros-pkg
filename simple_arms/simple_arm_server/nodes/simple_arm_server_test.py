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

usage_= "usage: simple_arm_server_test.py x y z wrist_pitch [wrist_roll=0.0 frame_id='base_link' duration='2.5']"

import roslib; roslib.load_manifest('simple_arm_server')
import rospy
import sys

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from simple_arm_server.srv import * 

if __name__ == '__main__':
    if len(sys.argv) > 4:     
        rospy.init_node('simple_arm_server_test')
        rospy.wait_for_service('simple_arm_server/move')
        move_srv = rospy.ServiceProxy('simple_arm_server/move', MoveArm) 

        req = MoveArmRequest()  
        req.pose_stamped.header.frame_id = "base_link"
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

        if len(sys.argv) > 7:
            req.move_time = rospy.Duration(float(sys.argv[7]))

        try:
            r = move_srv(req)
            print r
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)
    else:
        print usage_
