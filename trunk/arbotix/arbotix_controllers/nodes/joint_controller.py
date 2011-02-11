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

        # subscriptions
        rospy.Subscriber('cmd_joints', JointState, self.cmdJointsCb)
        
        # publishers
        self.publishers = dict()
        for name in self.joints:
            self.publishers[name] = rospy.Publisher(name+"/command", Float64)

        rospy.loginfo("Started joint_controller controlling: " + str(self.joints))
        rospy.spin()

    def cmdJointsCb(self, msg):
        """ The callback that converts JointState into servo movement. """
        for joint in msg.name:
            if joint in self.joints:            
                self.publishers[joint].publish(Float64(msg.position[msg.name.index(joint)]))

if __name__=="__main__":
    jc = JointController()

