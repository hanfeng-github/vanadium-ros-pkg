#!/usr/bin/env python

"""
  nuke_controller.py - controller for a NUKE walker
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

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

class base_controller(Thread):
    """ Controller to handle movement & odometry feedback for NUKE walker. """
    def __init__(self, device):
        Thread.__init__ (self)

        # handle for robocontroller
        self.device = device

        # parameters
        self.rate = float(rospy.get_param("~nuke_rate",15.0))
        
        # internal data        
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        self.then = datetime.now()  # time for determining dx/dy
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            now = datetime.now()
            elapsed = now - self.then
            self.then = now
            elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000.

            # read encoders
            try:
                x, y, th = self.device.getNukeEncoders()
            except:
                rospy.logerr("Could not update encoders")

            # calculate odometry
            dx = x / elapsed
            dy = y / elapsed
            dth = th / elapsed

            if x != 0 or y != 0:
                x2 = cos(th)*x - sin(th)*y
                y2 = -sin(th)*d + cos(th)*y
                self.x = self.x + (cos(self.th)*x2 - sin(self.th)*y2)
                self.y = self.y + (sin(self.th)*x2 + cos(self.th)*y2)

            if th != 0:
                self.th = self.th + th

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
            odom.twist.twist.linear.x = dx
            odom.twist.twist.linear.y = dy
            odom.twist.twist.angular.z = dth

            self.odomPub.publish(odom)

            r.sleep()

    def cmdVelCb(self,req):
        """ Handle movement requests. """
        x = req.linear.x*1000   # mm/s
        y = req.linear.y*1000   # mm/s
        th = req.angular.z*1000 # 1/1000 rad/s

        # clean up and log values
        rospy.loginfo("Twist move: "+str(x)+","+str(y)+","+str(th))
        x = int(x); y = int(y); th = int(th)
        # set motor speeds in ticks per 1/10s
        self.device.setWalk(x,y,th)  

