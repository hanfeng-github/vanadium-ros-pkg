#!/usr/bin/env python

"""
  nuke.py - controller for a walking robot
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

import rospy

from math import sin,cos,pi
from datetime import datetime   # TODO remove this dependency!

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

class NukeController:
    """ Controller to handle movement & odometry feedback for a walking robot. """
    def __init__(self, device):

        # handle for robocontroller
        self.device = device

        # parameters: rate
        self.throttle = int(device.rate/rospy.get_param("~nuke/rate",10))

        # internal data            
        self.enc_X = 0              # encoder readings
        self.enc_R = 0
        self.enc_Y = 0
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        self.then = datetime.now()  # time for determining dx/dy

        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.odomPub = rospy.Publisher("odom",Odometry)
        self.odomBroadcaster = TransformBroadcaster()
		
        rospy.loginfo("Started nuke_controller")

    def update(self):
        now = datetime.now()
        elapsed = now - self.then
        self.then = now
        elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000.

        # read encoders
        try:
            X,Y,R = self.device.getNukeEncoders()
        except:
            rospy.logerr("Could not update nuke encoders")
            return

        # calculate odometry
        dx = (X - self.enc_X)/1000.0
        dy = (Y - self.enc_Y)/1000.0
        dth = (R - self.enc_R)/1000.0
        self.enc_X = X
        self.enc_Y = Y
        self.enc_R = R

        self.x = self.x + (cos(self.th)*dx - sin(self.th)*dy)
        self.y = self.y + (sin(self.th)*dx + cos(self.th)*dy)
        self.th = self.th + dth

        quaternion = Quaternion()
        quaternion.x = 0.0 
        quaternion.y = 0.0
        quaternion.z = sin(self.th/2)
        quaternion.w = cos(self.th/2)
        # publish or perish
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0), 
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            "base_link",
            "odom"
            )

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = dx/elapsed
        odom.twist.twist.linear.y = dy/elapsed
        odom.twist.twist.angular.z = dth/elapsed
        self.odomPub.publish(odom)
 
    def startup(self):
        pass
    
    def shutdown(self):
        self.device.setWalk(0,0,0)

    def cmdVelCb(self,req):
        """ Handle movement requests. """
        x = int(req.linear.x * 1000)   # m/s   --> mm/s
        y = int(req.linear.y * 1000)   # m/s   --> mm/s
        th = int(req.angular.z *1000)  # rad/s --> 1/1000 rad/s
        print "walk", x, y, th
        self.device.setWalk(x,y,th)

