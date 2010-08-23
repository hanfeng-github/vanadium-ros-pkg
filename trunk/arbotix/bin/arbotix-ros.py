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

from arbotix.arbotix import ArbotiX # does this look ridiculous to anyone else?
from arbotix.srv import *
from arbotix.ax12 import *

# TODO: generalize these, add init.py in packages
from arbotix_sensors.lidar import *
from arbotix_controllers.base_controller import *

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
                self.max_speed = radians(float(params[key]))
            elif key=='max_angle':
                self.max_angle = radians(float(params[key]))
            elif key=='min_angle':
                self.min_angle = radians(float(params[key]))
            elif key=='ticks':
                self.ticks = int(params[key])
            elif key=='range':
                self.rad_per_tick = radians(float(params[key]))/self.ticks
            elif key=='neutral':
                self.neutral = params[key]
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
            rospy.logerr("Servo "+self.name+": angle out of range ("+str(ang)+"). Limits are: " + str(self.min_angle) + "," + str(self.max_angle))  
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

class ArbotiX_ROS(ArbotiX):
    
    def __init__(self):
        rospy.init_node('arbotix')
        # load configurations    
        port = rospy.get_param("~port", "/dev/ttyUSB0")                     
        baud = int(rospy.get_param("~baud", "38400"))
        rospy.loginfo("Starting ArbotiX-ROS on port "+port)

        # start an arbotix driver
        ArbotiX.__init__(self, port, baud)        

        # initialize servos and state publishing
        use_sync = rospy.get_param("~use_sync",True)                        # use sync read?
        use_single = rospy.get_param("~use_single_services",False)          # use single read/write services?
        dynamixels = rospy.get_param("~dynamixels", dict())
        self.dynamixel_servos = dict()
        self.sync_servos = list()
        self.sync_names = list()
        for index in dynamixels.keys():
            servo = DynamixelServo(int(index), dynamixels[index], self, use_single) 
            self.dynamixel_servos[servo.name] = servo 
            self.sync_servos.append(servo.id)    
            self.sync_names.append(servo.name)    

        servos = rospy.get_param("~servos", dict())
        self.hobby_servos = list()
        for index in servos.keys():
            self.hobby_servos.append( HobbyServo(int(index), servos[index], self) )

        self.jointStatePub = rospy.Publisher('joint_states', JointState)

        # initialize digital/analog IO
        rospy.Service('GetDigital',GetDigital,self.getDigitalCb)   
        rospy.Service('GetAnalog',GetAnalog,self.getAnalogCb)
        rospy.Service('SetDigital',SetDigital,self.setDigitalCb)

        # initialize controllers
# TODO: START COMPLETE REWRITE
        use_base = rospy.get_param("~use_base",False)                       # use closed-loop base?
        use_nuke = rospy.get_param("~use_nuke",False)                       # use nuke base?
        use_lidar = rospy.get_param("~use_lidar",False)                     # use lidar?

#        # for command callbacks
#        #rospy.Subscriber("command", JointTrajectory, self.commandCb)
#        #self.servo_trajectories = dict()    

        # listen for movement commands, send to robot
        if use_base == True:
            #self.base = ArbotiXMobileBase(self.device, ticks_meter, base_width)
            pass
        if use_nuke == True:
            # TODO
            pass    

        # Poor Man's LIDAR
        if use_lidar == True:
            self.lidar = lidar(self)    
            self.lidar.start()
# TODO: END COMPLETE REWRITE

        # publish joint states (everything else is a service/topic callback)
        r = rospy.Rate(int(rospy.get_param("~rate",10)))
        while not rospy.is_shutdown():
                
            # publish joint states
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

if __name__ == "__main__":
    a = ArbotiX_ROS()

