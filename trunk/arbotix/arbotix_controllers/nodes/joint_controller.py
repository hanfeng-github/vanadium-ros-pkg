#!/usr/bin/env python

""" 
  joint_controller.py - the simplest joint control method known to mankind
  Copyright (c) 2010-2011 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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

