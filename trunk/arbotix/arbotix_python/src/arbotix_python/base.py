#!/usr/bin/env python

"""
  base.py - controller for a differential drive
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

class BaseController:
    """ Controller to handle movement & odometry feedback for a differential 
            drive mobile base. """
    def __init__(self, device):

        # handle for robocontroller
        self.device = device

        # parameters: throttle rate and geometry
        self.throttle = int(device.rate/rospy.get_param("~base/rate",10))
        self.ticks_meter = float(rospy.get_param("~base/ticks_meter", 26154))
        self.base_width = float(rospy.get_param("~base/base_width", 0.144))

        # parameters: PID
        self.Kp = rospy.get_param("~base/Kp", 5)
        self.Kd = rospy.get_param("~base/Kd", 1)
        self.Ki = rospy.get_param("~base/Ki", 0)
        self.Ko = rospy.get_param("~base/Ko", 50)       

        # parameters: acceleration
        self.accel_limit = rospy.get_param("~base/accel_limit", 0.1)
        self.max_accel = int(self.accel_limit*self.ticks_meter/(device.rate/self.throttle))

        # internal data            
        self.v_left = 0             # current setpoint velocity
        self.v_right = 0
        self.v_des_left = 0         # cmd_vel setpoint
        self.v_des_right = 0
        self.enc_left = 0           # encoder readings
        self.enc_right = 0
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        self.then = datetime.now()  # time for determining dx/dy

        # output for joint states
        self.names = ["base_l_wheel_joint","base_r_wheel_joint"]
        self.positions = [0,0]

        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.odomPub = rospy.Publisher("odom",Odometry)
        self.odomBroadcaster = TransformBroadcaster()
		
        rospy.loginfo("Started base_controller for a base of " + str(self.base_width) + "m wide with " + str(self.ticks_meter) + " ticks per meter")

    def update(self):
        now = datetime.now()
        elapsed = now - self.then
        self.then = now
        elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000.

        # read encoders
        try:
            left, right = self.device.getEncoders()
        except:
            rospy.logerr("Could not update encoders")
            return
        rospy.logdebug("Encoders: " + str(left) +","+ str(right))

        # calculate odometry
        d_left = (left - self.enc_left)/self.ticks_meter
        d_right = (right - self.enc_right)/self.ticks_meter
        self.enc_left = left
        self.enc_right = right

        d = (d_left+d_right)/2
        th = (d_right-d_left)/self.base_width
        dx = d / elapsed
        dth = th / elapsed

        if (d != 0):
            x = cos(th)*d
            y = -sin(th)*d
            self.x = self.x + (cos(self.th)*x - sin(self.th)*y)
            self.y = self.y + (sin(self.th)*x + cos(self.th)*y)
        if (th != 0):
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
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = dth
        self.odomPub.publish(odom)

        # update motors
        if self.v_left < self.v_des_left:
            self.v_left += self.max_accel
            if self.v_left > self.v_des_left:
                self.v_left = self.v_des_left
        else:
            self.v_left -= self.max_accel
            if self.v_left < self.v_des_left:
                self.v_left = self.v_des_left
        
        if self.v_right < self.v_des_right:
            self.v_right += self.max_accel
            if self.v_right > self.v_des_right:
                self.v_right = self.v_des_right
        else:
            self.v_right -= self.max_accel
            if self.v_right < self.v_des_right:
                self.v_right = self.v_des_right
        self.device.setSpeeds(self.v_left, self.v_right)
 
    def startup(self):
        self.device.write(253,self.device.KP,[self.Kp,self.Kd,self.Ki,self.Ko]) 
    
    def shutdown(self):
        self.device.setSpeeds(0,0)

    def cmdVelCb(self,req):
        """ Handle movement requests. """
        x = req.linear.x        # m/s
        th = req.angular.z      # rad/s

        if x == 0:
            # turn in place
            r = th * self.base_width/2.0 * self.ticks_meter
            l = -r
        elif th == 0:   
            # pure forward/backward motion
            l = r = x * self.ticks_meter
        else:
            # rotation about a point in space
            l = (x - th * self.base_width/2.0) * self.ticks_meter
            r = (x + th * self.base_width/2.0) * self.ticks_meter

        # clean up and log values                  
        rospy.logdebug("Twist move: "+str(l)+","+str(r))
        l = int(l/30.0)
        r = int(r/30.0)      
        # set motor speeds in ticks per 1/30s
        self.v_des_left = l
        self.v_des_right = r

