#!/usr/bin/env python

""" 
  joint_controller.py - the simplest joint control method known to mankind
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

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

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointController():
    """ Controller to handle basic servo control. """

    def __init__(self):
        # initialize a node
        rospy.init_node("joint_controller")
        self.joints = rospy.get_param("~joints")
        self.rate = rospy.get_param("~rate", 50)

        # joint values
        self.valid = False
        self.dirty = False
        self.current = [0.0 for i in self.joints]
        self.desired = [0.0 for i in self.joints]   
        self.velocity = [0.0 for i in self.joints]         

        # subscribers and publishers
        rospy.Subscriber('cmd_joints', JointState, self.cmdJointsCb)
        rospy.Subscriber('joint_states', JointState, self.stateCb)
        self.publishers = dict(zip(self.joints, [rospy.Publisher(name+"/command", Float64) for name in self.joints]))

        rospy.loginfo("Started joint_controller controlling: " + str(self.joints))

        rate = rospy.Rate(self.rate)
        cumulative = 0.0
        while not rospy.is_shutdown():
            if self.dirty and self.valid:
                err = [ (d-c) for d,c in zip(self.desired,self.current)]
                for i in range(len(self.joints)):
                    cmd = err[i]
                    cumulative += cmd
                    if self.velocity[i] != 0:
                        top = self.velocity[i]/self.rate
                        if cmd > top:
                            cmd = top
                        elif cmd < -top:
                            cmd = -top
                    self.current[i] += cmd
                    self.publishers[self.joints[i]].publish(Float64(self.current[i]))
                if cumulative == 0.0:
                    self.dirty = False
            rate.sleep()

    def cmdJointsCb(self, msg):
        """ The callback that converts JointState into servo movement. """
        try:
            indexes = [msg.name.index(name) for name in self.joints]
        except ValueError as val:
            self.dirty = False
            rospy.logerr('Invalid joint state message.')
            return            
        self.velocity = [ msg.velocity[k] for k in indexes ]
        self.desired = [ msg.position[k] for k in indexes ]
        self.dirty = True

    def stateCb(self, msg):
        """ The callback that converts JointState into servo position for interpolation. """
        if self.dirty and self.valid:
            return
        try:
            indexes = [msg.name.index(name) for name in self.joints]
        except ValueError as val:
            self.valid = False
            rospy.logerr('Invalid joint state message.')
            return           
        self.current = [ msg.position[k] for k in indexes ]
        self.valid = True

if __name__=="__main__":
    jc = JointController()

