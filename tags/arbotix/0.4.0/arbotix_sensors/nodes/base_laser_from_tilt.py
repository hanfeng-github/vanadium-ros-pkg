#!/usr/bin/env python

"""
  base_laser_from_tilt.py - extract a base scan from a laser on a tilt stage
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

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

import roslib; roslib.load_manifest('arbotix_sensors')
import rospy

from sensor_msgs.msg import LaserScan, JointState

class base_laser_from_tilt():
    """ Create a base scan from a tilting laser. """

    def __init__(self):
        # parameters
        self.joint = rospy.get_param("~joint","laser_tilt_mount_joint")
        self.copy_position = rospy.get_param("~copy_position",0.0)
        self.copy_threshold = rospy.get_param("~copy_threshold",0.05)
        self.tilt_scan = rospy.get_param("~tilt_scan","tilt_scan")
        self.new_scan = rospy.get_param("~base_scan","base_scan")
        self.new_frame_id = rospy.get_param("~base_frame","base_laser_link") 

        # publisher
        self.scanPub = rospy.Publisher("base_scan", LaserScan)
        rospy.Subscriber(self.tilt_scan, LaserScan, self.laserCb)
        rospy.Subscriber('joint_states', JointState, self.stateCb)

        self.copy = False

        rospy.loginfo("Started base_laser sensor '"+name)

    def laserCb(self, msg):
        if self.copy:
            msg.header.frame_id = self.new_frame_id
            self.scanPub.publish(msg)
            print "Published Base Scan"
            self.copy = False

    def stateCb(self, msg):
        try:
            if abs(msg.position[msg.name.index(self.joint)] - self.copy_position) < self.copy_threshold:
                self.copy = True
        except:
            self.copy = False

if __name__ == "__main__":
    rospy.init_node("base_laser_from_tilt")
    node = base_laser_from_tilt()
    rospy.spin()

