#!/usr/bin/env python

# TODO: Base movement, Nuke movement
# Move gp_lidar distance calcs down to the ServoStiK

"""
  ArbotiX ROS Node: serial connection to an ArbotiX board w/ PyPose/NUKE/ROS
  Copyright (c) 2008-2010 Michael E. Ferguson.  All right reserved.

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

import roslib; roslib.load_manifest('arbotix')
import rospy

from sensor_msgs.msg import JointState
#from trajectory_msgs.msg import JointTrajectory

from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from arbotix.arbotix import ArbotiX # does this look ridiculous to anyone else?
from arbotix_sensors.lidar import *
from arbotix.srv import *
from arbotix.ax12 import *

from math import sin,cos,pi,radians
from datetime import datetime

###############################################################################
# Servo handling classes    
class DynamixelServo():
    """ Class to handle services and updates for a single Dynamixel Servo, on 
        an ArbotiX robocontroller's AX/RX-bus. """
    def __init__(self, index, params, device, single=False):
        self.id = index
        self.device = device                        # ArbotiX instance

        self.neutral = 512                          # adjust for EX-106, etc
        self.ticks = 1024                           # adjust for EX-106, etc
        self.rad_per_tick = radians(300.0)/1024     # adjust for EX-106, etc
        self.max_angle = radians(150)               # limit angle, radians
        self.min_angle = radians(-150)
        self.max_speed = radians(1)                 # radians per second
        self.invert = False
        self.setParams(params)

        self.angle = 0.0                            # current position

        # some callbacks
        if single:
            self.srvRead = rospy.Service(self.name+'_getangle',GetAngle, self.getAngleCb)
            self.srvMoving = rospy.Service(self.name+'_ismoving',IsMoving, self.isMovingCb)
            self.srvWrite = rospy.Service(self.name+'_setangle',SetAngle, self.setAngleCb)
        # this will get indented once SetJoints stuff works
        self.srvRelax = rospy.Service(self.name+'_relax',Relax, self.relaxCb)

    def setParams(self, params):
        for key in params.keys():
            if key=='invert':
                if int(params[key]) > 0:
                    self.invert = True
            elif key=='max_speed':
                self.max_speed = radians(params[key])
            elif key=='max_angle':
                self.max_angle = radians(params[key])
            elif key=='min_angle':
                self.max_angle = radians(params[key])
            elif key=='ticks':
                self.ticks = int(params[key])
            elif key=='range':
                self.rad_per_tick = radians(float(params[key]))/self.ticks
            elif key=='neutral':
                self.neutral = param[key]
            elif key=='name':
                self.name = params[key]
            else:
                rospy.logerr("Parameter '" + key + "' not recognized.")

    def setAngleCb(self, req):
        self.setAngle(req.angle)
        return SetAngleResponse()

    def getAngleCb( self, req ):
        """ ROS service to get angle of servo (in radians) """
        return GetAngleResponse( self.getAngle() )

    def isMovingCb(self, req):
        status = self.device.read(self.id, P_MOVING, 1)[0]
        return IsMovingResponse( int(status) )

    def relaxCb(self, req):
        """ Turn off servo torque, so that it is pose-able. """
        self.device.disableTorque(self.id)
        return RelaxResponse()

    def setAngle(self, ang):
        if ang > self.max_angle or ang < self.min_angle:
            rospy.logerr("Servo "+self.name+": angle out of range ("+str(ang)+")")  
            return 
        self.angle = ang    # store it for joint state updates
        if self.invert:
            ang = ang * -1.0
        ticks = int(round( ang / self.rad_per_tick ))
        ticks += int(self.neutral)
        self.device.setPosition(self.id, ticks)
        
    def getAngle(self, pos=None):
        """ Find angle in radians by reading from servo, or
            by using pos passed in from a sync read.  """
        if pos == None:
            pos = self.device.getPosition(self.id)
        if pos != -1:
            angle = (pos - self.neutral) * self.rad_per_tick
            if self.invert:
                angle = angle * -1.0
            self.angle = angle
        return self.angle

class HobbyServo(DynamixelServo):
    """ Class to handle services and updates for a single Hobby Servo, connected to 
        an ArbotiX robocontroller. A stripped down version of the DynamixelServo. """
    def __init__(self, index, params, device, single=False):
        self.id = index
        self.device = device                        # ArbotiX instance

        self.neutral = 1500                         # might be adjusted for crappy servos
        self.ticks = 2000
        self.rad_per_tick = radians(180.0)/2000     # 180 degrees over 500-2500ms 
        self.max_angle = radians(90)                # limit angle, radians
        self.min_angle = radians(-90)
        self.invert = False
        self.setParams(params)

        self.angle = 0.0                            # current position

        # a callback
        if single: 
            self.srvWrite = rospy.Service(self.name+'_setangle',SetAngle, self.setAngleCb)

    def setAngleCb(self, req):
        """ Callback to set position to angle, in radians. """
        ang = req.angle
        if ang > self.max_angle or ang < self.min_angle:
            rospy.logerr("Servo "+self.name+": angle out of range ("+str(ang)+")")            
            return SetAngleResponse()
        self.angle = ang    # store it for joint state updates
        if self.invert:
            ang = ang * -1.0
        ticks = int(round( ang / self.rad_per_tick ))
        ticks += self.neutral
        rospy.loginfo("Servo "+self.name+": set to "+str(ticks))
        self.device.setServo(self.id, ticks)
        return SetAngleResponse()        

    def getAngle(self):
        """ Find angle in radians """
        return self.angle

class ArbotiXMobileBase():
    """ Class to handle services, odometry feedback, etc, for a mobile base, 
        powered by an ArbotiX RoboController. """
    def __init__(self, device, ticks_meter, base_width):
        self.device = device        # handle for robocontroller
        self.ticks_meter = ticks_meter
        self.base_width = base_width

        rospy.Subscriber("cmd_vel", Twist, self.twist)
        self.odomPub = rospy.Publisher('odom',Odometry)
        self.odomBroadcaster = TransformBroadcaster()

#        self.ticks_rad = (base_width*ticks_meter) / (2.0*pi)
        self.enc_left = 0           # encoder readings
        self.enc_right = 0
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        self.then = datetime.now()  # time for determining dx/dy

    def update(self):
        now = datetime.now()
        elapsed = now - self.then
        self.then = now
        elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000.

        left = self.device.getLenc()
        right = self.device.getRenc()
        #
        dl = left - self.enc_left
        dr = right - self.enc_right

        # convert 
        d = 0   #d = self.create.d_distance / 1000.
        th = 0  #th = self.create.d_angle*pi/180
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

    def twist(self,req):
        """ Handle movement requests. """
        x = req.linear.x        # m/s
        th = req.angular.z      # rad/s

        # if theta were 0
        l = (x/30.0 * self.ticks_meter) - (th/30.0 * self.ticks_rad)
        r = (x/30.0 * self.ticks_meter) + (th/30.0 * self.ticks_rad)

        l = int(l)
        r = int(r)

        rospy.loginfo("Twist move: "+str(l)+","+str(r))
        if l == 0 and r == 0:
            self.device.SetLeftSpeed(0)
            self.device.SetRightSpeed(0)
        else:
            self.device.SetLeftSpeed(int(l))
            self.device.SetRightSpeed(int(r))

class ArbotiX_ROS(ArbotiX):
    
    def __init__(self):
        rospy.init_node('arbotix')
        # load configurations    
        port = rospy.get_param("~port", "/dev/ttyUSB0")                     
        baud = int(rospy.get_param("~baud", "38400"))
        rospy.loginfo("Starting ArbotiX-ROS on port "+port)
        dynamixels = rospy.get_param("~dynamixels", dict())
        servos = rospy.get_param("~servos", dict())
        ticks_meter = float(rospy.get_param("~ticks_meter", "26154"))       # ticks per meter of wheel rotation
        base_width = float(rospy.get_param("~base_width", "0.144"))         # width of base (in meters), for angular movement
        use_sync = rospy.get_param("~use_sync",True)                        # use sync read?
        use_base = rospy.get_param("~use_base",False)                       # use closed-loop base?
        use_nuke = rospy.get_param("~use_nuke",False)                       # use nuke base?
        use_gp_lidar = rospy.get_param("~use_gp_lidar",False)               # use gp lidar?
        
        # start an arbotix driver
        ArbotiX.__init__(self, port, baud)

        # initialize servos and state publishing
        self.dynamixel_servos = dict()
        self.sync_servos = list()
        self.sync_names = list()
        for index in dynamixels.keys():
            servo = DynamixelServo(int(index), dynamixels[index], self) 
            self.dynamixel_servos[servo.name] = servo 
            self.sync_servos.append(servo.id)    
            self.sync_names.append(servo.name)    

        self.hobby_servos = list()
        for index in servos.keys():
            self.hobby_servos.append( HobbyServo(int(index), servos[index], self) )

        # for command callbacks
        #rospy.Subscriber("command", JointTrajectory, self.commandCb)
        self.servo_trajectories = dict()    

        # and output of current positions
        self.jointStatePub = rospy.Publisher('joint_states', JointState)
    
        # digital/analog IO
        rospy.Service('GetDigital',GetDigital,self.getDigitalCb)   
        rospy.Service('GetAnalog',GetAnalog,self.getAnalogCb)
        rospy.Service('SetDigital',SetDigital,self.setDigitalCb)

        # listen for movement commands, send to robot
        if use_base == True:
            self.base = ArbotiXMobileBase(self.device, ticks_meter, base_width)
        if use_nuke == True:
            # TODO
            pass    

        # Poor Man's LIDAR
        if use_lidar == True:
            self.lidar = lidar(self)    
            self.lidar.start()

        # publish joint states (everything else is a service callback)
        r = rospy.Rate(int(rospy.get_param("~rate",10)))
        while not rospy.is_shutdown():
            # update our output values
            #for servo in self.servo_trajectories.keys():
            
            queue = self.servo_queue    
            self.servo_queue = dict()
            for servo, position in queue:   
                # TODO: convert to sync write
                dynamixel_servo[servo].setAngle(position)         
                
            # now publish joint states
            msg = JointState()
            msg.name = list()
            msg.position = list()
            msg.velocity = list()
            msg.effort = list()

            if use_sync: 
                # arbotix/servostik/wifi board sync_read
                val = self.syncRead(self.sync_servos, P_PRESENT_POSITION_L, 2)
                if val != None:            
                    i = 0        
                    for servo in self.sync_names:
                        msg.name.append(self.dynamixel_servos[servo].name)
                        msg.position.append(self.dynamixel_servos[servo].getAngle( val[i]+(val[i+1]<<8) ))
                        i = i + 2
            else:
                # direct connection, or other hardware with no sync_read capability
                for servo in self.dynamixel_servos.values():
                    msg.name.append(servo.name)
                    msg.position.append(servo.getAngle())  
                
            for servo in self.hobby_servos:
                msg.name.append(servo.name)
                msg.position.append(servo.getAngle())  

            msg.header.stamp = rospy.Time.now()
            self.jointStatePub.publish(msg)                  
            r.sleep()

    def getDigitalCb(self, req):
        return GetDigitalResponse( self.getDigital(req.pin) )

    def getAnalogCb(self, req):    
        return GetAnalogResponse( self.getAnalog(req.pin) )

    def setDigitalCb(self, req):
        self.setDigital(req.pin, req.dir, req.value)
        return SetDigitalResponse()

    def commandCb(self, req):
        # trajectories["name"] --> [ [time,position,velocity], [time,position,velocity], ....]
        for i in len(req.joint_names):
            servo = req.joint_names[i]
            t = self.servo_trajectories[servo]
            # remove trajectories we're tossing 
            while len(t) > 0 and t[len(t)-1][0] > req.header.stamp:
                t.pop()
            # add new trajectories
            for point in req.points[i]:
                t.append( [req.header.stamp + point.time_from_start, point.positions[i], point.velocities[i]] )
            # post the update
            self.servo_trajectories[servo] = t                            

if __name__ == "__main__":
    a = ArbotiX_ROS()

