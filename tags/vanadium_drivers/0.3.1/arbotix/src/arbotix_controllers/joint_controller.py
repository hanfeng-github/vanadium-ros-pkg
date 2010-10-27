#!/usr/bin/env python

"""
  joint_controller.py - the simplest joint control method known to mankind
  Copyright (c) 2010 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
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

import rospy
from threading import Thread

from sensor_msgs.msg import JointState

class joint_controller(Thread):
    """ Controller to handle basic servo control. """

    def __init__(self, device, name, joints=None):
        Thread.__init__ (self)

        # handle for robocontroller
        self.device = device
        self.name = name

        # parameters
        if joints==None:
            self.joints = rospy.get_param("~controllers/"+name+"/joints")
        else: 
            # being used as default controller
            self.joints = joints        

        # subscriptions
        rospy.Subscriber('cmd_joints', JointState, self.cmdJointsCb)

        rospy.loginfo("Started joint_controller '"+name+"' controlling: " + str(self.joints))

    def run(self):
        """ Simply for compliance with our controller model. """
        rospy.spin()

    def cmdJointsCb(self, msg):
        """ The callback that converts JointState into servo movement. """
        for joint in msg.name:
            if joint in self.joints:            
                # TODO: sync this!
                self.device.servos[joint].setAngle( msg.position[msg.name.index(joint)] )

