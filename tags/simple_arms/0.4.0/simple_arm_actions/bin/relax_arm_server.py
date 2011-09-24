#!/usr/bin/env python

"""
  relax_arm_server.py - a simple service to relax the arm
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

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

import roslib; roslib.load_manifest('arbotix_python'); roslib.load_manifest('simple_arm_actions')
import rospy, actionlib

from arbotix_msgs.srv import Relax
from simple_arm_actions.msg import *

class RelaxArmServer:

    def __init__(self):
        rospy.init_node("relax_arm_server")
        try:
            self.joints = rospy.get_param("~joints")
        except:
            self.joints = rospy.get_param("/simple_arms/joints")

        self.services = [rospy.ServiceProxy(name+'/relax',Relax) for name in self.joints]
        self.server = actionlib.SimpleActionServer("relax_arm", ResetArmAction, execute_cb=self.actionCb, auto_start=False)
        self.server.start()
        rospy.spin()

    def actionCb(self, req):
        for service in self.services:
            service()
        self.server.set_succeeded( ResetArmResult() )   


if __name__ == '__main__':
    try:
        RelaxArmServer()
    except rospy.ROSInterruptException:
        rospy.loginfo("And that's all folks...")

