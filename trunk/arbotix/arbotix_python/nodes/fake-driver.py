#!/usr/bin/env python

"""
  ArbotiX Node: Fake ArbotiX!
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

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

import roslib; roslib.load_manifest('arbotix_python')
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from arbotix_msgs.msg import *
from arbotix_msgs.srv import *
from datetime import datetime   # TODO remove this dependency!

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from math import radians,sin,cos,pi

###############################################################################
# Servo handling classes    
class Servo():
    """ Class to handle services and updates for a single Servo, on an ArbotiX 
        robocontroller's AX/RX-bus. """
    def __init__(self, name, device):
        self.name = name
        self.device = device           # ArbotiX instance
        n = "~dynamixels/"+name+"/"

        # TODO: load URDF specs

        self.id = int(rospy.get_param(n+"id"))
        self.neutral = rospy.get_param(n+"neutral",512)
        self.ticks = rospy.get_param(n+"ticks",1024)
        self.rad_per_tick = radians(rospy.get_param(n+"range",300.0))/self.ticks

        self.max_angle = radians(rospy.get_param(n+"max_angle",150))
        self.min_angle = radians(rospy.get_param(n+"min_angle",-150))
        self.max_speed = radians(rospy.get_param(n+"max_speed",684.0)) 
                                       # max speed = 114 rpm - 684 deg/s
        self.invert = rospy.get_param(n+"invert",False)
        self.readable = rospy.get_param(n+"readable",True)

        self.dirty = False             # newly updated position?
        self.angle = 0.0               # current position, as returned by servo (radians)
        self.desired = 0.0             # desired position (radians)
        self.last_cmd = 0.0            # last position sent (radians)
        self.velocity = 0.0            # moving speed
        self.relaxed = True            # are we not under torque control?
        self.last = rospy.Time.now()
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)
        #rospy.Service(name+'/relax', Relax, self.relaxCb)

    def relaxCb(self, req):
        """ Turn off servo torque, so that it is pose-able. """
        self.device.disableTorque(self.id)
        self.relaxed = True
        return RelaxResponse()

    def commandCb(self, req):
        self.dirty = True   
        self.relaxed = False
        self.desired = req.data
   
    def update(self, value):
        """ Update angle in radians by reading from servo, or
            by using position passed in from a sync read.  """
        if value < 0:
            # read servo locally (no sync_read)
            if self.readable:
                value = self.device.getPosition(self.id)
        if value != -1:
            # convert ticks to radians
            last_angle = self.angle
            if self.invert:
                self.angle = -1.0 * (value - self.neutral) * self.rad_per_tick
            else:
                self.angle = (value - self.neutral) * self.rad_per_tick
            # update velocity estimate
            t = rospy.Time.now()
            self.velocity = (t - self.last).to_nsec()/1000000000.0
            self.last = t
        if self.relaxed:
            self.last_cmd = self.angle

    def interpolate(self, frame):
        """ Get the new position to move to, in ticks. """
        if self.dirty:
            # compute command, limit velocity
            cmd = self.desired - self.last_cmd
            if cmd > self.max_speed/float(frame):
                cmd = self.max_speed/float(frame)
            elif cmd < -self.max_speed/float(frame):
                cmd = -self.max_speed/float(frame)
            # compute angle, apply limits
            self.last_cmd += cmd
            if self.last_cmd < self.min_angle:
                self.last_cmd = self.min_angle
            if self.last_cmd > self.max_angle:
                self.last_cmd = self.max_angle
            self.speed = cmd*frame
            # cap movement
            if self.last_cmd == self.desired:
                self.dirty = False
            return self.neutral + (self.last_cmd/self.rad_per_tick)
        else:
            return None

class HobbyServo(Servo):
    """ Class to handle services and updates for a single Servo, on an ArbotiX 
        robocontroller's AX/RX-bus. """
    def __init__(self, name, device):
        self.name = name
        self.device = device           # ArbotiX instance
        n = "~servos/"+name+"/"

        # TODO: load URDF specs

        self.id = int(rospy.get_param(n+"id"))
        self.neutral = rospy.get_param(n+"neutral",512)
        self.ticks = rospy.get_param(n+"ticks",1024)
        self.rad_per_tick = radians(rospy.get_param(n+"range",300.0))/self.ticks

        self.max_angle = radians(rospy.get_param(n+"max_angle",150))
        self.min_angle = radians(rospy.get_param(n+"min_angle",-150))
        self.max_speed = radians(rospy.get_param(n+"max_speed",684.0)) 
                                       # max speed = 114 rpm - 684 deg/s
        self.invert = rospy.get_param(n+"invert",False)
        self.readable = rospy.get_param(n+"readable",True)

        self.dirty = False             # newly updated position?
        self.angle = 0.0               # current position, as returned by servo (radians)
        self.desired = 0.0             # desired position (radians)
        self.last_cmd = 0.0            # last position sent (radians)
        self.velocity = 0.0            # moving speed
        self.relaxed = True            # are we not under torque control?
        self.last = rospy.Time.now()
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)
        #rospy.Service(name+'/relax', Relax, self.relaxCb)

###############################################################################
# Base handling class  
class BaseController:
    """ Controller to handle movement & odometry feedback for a differential 
            drive mobile base. """
    def __init__(self, device):

        # handle for robocontroller
        self.device = device

        # parameters: throttle rate and geometry
        self.throttle = int(device.rate/rospy.get_param("~base/rate",10))

        self.dx = 0                 # cmd_vel setpoint
        self.dr = 0

        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        self.then = datetime.now()  # time for determining dx/dy

        # output for joint states
        self.names = ["base_l_wheel_joint","base_r_wheel_joint"]
        self.positions = [0,0]
        self.velocities = [0,0]

        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.odomPub = rospy.Publisher("odom",Odometry)
        self.odomBroadcaster = TransformBroadcaster()
		
        rospy.loginfo("Started base_controller")

    def update(self):
        now = datetime.now()
        elapsed = now - self.then
        self.then = now
        elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000.

        x = cos(self.th)*self.dx*elapsed
        y = -sin(self.th)*self.dx*elapsed
        self.x += cos(self.th)*self.dx*elapsed
        self.y += sin(self.th)*self.dx*elapsed
        self.th += self.dr*elapsed

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
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)

    def cmdVelCb(self,req):
        """ Handle movement requests. """
        self.dx = req.linear.x        # m/s
        self.dr = req.angular.z       # rad/s

###############################################################################
# Main ROS interface
class ArbotixROS():
    
    def __init__(self):
        rospy.init_node('arbotix')
        # load configurations    
        self.rate = int(rospy.get_param("~rate", 100))
        self.throttle_r = int(self.rate/rospy.get_param("~read_rate", 10)) # throttle rate for read
        self.throttle_w = int(self.rate/rospy.get_param("~write_rate", 10)) # throttle rate for write
        self.use_sync  = rospy.get_param("~use_sync",True) # use sync read?

        # start an arbotix driver
        #ArbotiX.__init__(self, port, baud)        
        rospy.sleep(1.0)
        rospy.loginfo("Started FAKE ArbotiX")
        
        # initialize dynamixel & hobby servos
        dynamixels = rospy.get_param("~dynamixels", dict())
        self.dynamixels = dict()
        for name in dynamixels.keys():
            self.dynamixels[name] = Servo(name,self)
        hobbyservos = rospy.get_param("~servos", dict())
        self.servos = dict()
        for name in hobbyservos.keys():
            self.servos[name] = HobbyServo(name, self)

        # setup a base controller
        self.use_base = False
        if rospy.has_param("~base"):
            self.use_base = True
            self.base = BaseController(self)

        # publishers, subscribers and services
        self.jointStatePub = rospy.Publisher('joint_states', JointState)

        r = rospy.Rate(self.rate)
        f = 0  # frame ID
        # main loop -- do all the read/write here
        while not rospy.is_shutdown():
    
            # update servo positions (via sync_write)
            if f%self.throttle_w == 0:
            #    syncpkt = list()
                for servo in self.dynamixels.values():
                    v = servo.interpolate(self.rate/self.throttle_w)
                #for servo in self.servos.values():
                #    v = servo.interpolate(self.rate/self.throttle_w)
            #        if v != None:
            #            syncpkt.append([servo.id,int(v)%256,int(v)>>8])  
            #    if len(syncpkt) > 0:      
            #        self.syncWrite(P_GOAL_POSITION_L,syncpkt)

            # update base
            if self.use_base and f%self.base.throttle == 0:
                self.base.update()

            # publish joint states
            if f%self.throttle_r == 0:
                # publish joint states         
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = list()
                msg.position = list()
                msg.velocity = list()
                for servo in self.dynamixels.values():
                    servo.angle = servo.last_cmd          
                    msg.name.append(servo.name)
                    msg.position.append(servo.angle)
                    msg.velocity.append(servo.velocity)
                for servo in self.servos.values():
                    servo.angle = servo.last_cmd          
                    msg.name.append(servo.name)
                    msg.position.append(servo.angle)
                    msg.velocity.append(servo.velocity)
                if self.use_base:
                    msg.name += self.base.names
                    msg.position += self.base.positions
                    msg.velocity += self.base.velocities
                self.jointStatePub.publish(msg)

            f += 1
            r.sleep()     


if __name__ == "__main__":
    a = ArbotixROS()

