#!/usr/bin/env python

"""
  parallel_single_servo_controller.py - controls a single-servo parallel-jaw gripper
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
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

import roslib; roslib.load_manifest('arbotix_controllers')
import rospy
import thread

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from math import asin

class ParallelGripperController:
    """ A simple controller that operates a single servo parallel jaw gripper. """
    def __init__(self):
        rospy.init_node("gripper_controller")

        # TODO: load calibration data
        self.calib = { 0.0000 : 866, 0.0159: 750, 0.0254 : 688, 0.0381 : 600 }
        self.min = rospy.get_param("~min", 0.0)
        self.max = rospy.get_param("~max", 0.0381)

        #self.center = rospy.get_param("~center", 0.0)
        self.invert = rospy.get_param("~invert", False)
        
        # publishers
        self.commandPub = rospy.Publisher("gripper_joint/command", Float64)
        self.jointStatePub = rospy.Publisher('joint_states', JointState)

        # subscribe to command and then spin
        self.command = 0.0
        rospy.Subscriber("~command", Float64, self.commandCb)
        
        r = rospy.Rate(15)
        while not rospy.is_shutdown():
            # output joint state
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name = ["gripper_left_joint", "gripper_right_joint"]
            js.position = [-self.command/2.0,self.command/2.0]
            js.velocity = [0,0]
            #self.jointStatePub.publish(js) 
            r.sleep()

    def commandCb(self, msg):
        """ Take an input command of width to open gripper. """
        # check limits
        if msg.data > self.max or msg.data < self.min:
            rospy.logerr("Command exceeds limits.")
            return
        # compute angles
        low = self.min
        high = self.max
        for d in self.calib.keys():
            if self.calib[d] > self.min and self.calib[d] < msg.data:
                low = d
            if self.calib[d] < self.max and self.calib[d] > msg.data:
                high = d
        scale = msg.data/(high-low)
        value = ((self.calib[high] - self.calib[low]) * scale) + self.calib[low]
        self.commandPub.publish( Float64(value) )
        self.command = msg.data

if __name__=="__main__": 
    try:
        ParallelGripperController()
    except rospy.ROSInterruptException:
        rospy.loginfo("Hasta la Vista...")
        
