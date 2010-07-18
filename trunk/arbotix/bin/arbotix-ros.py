#!/usr/bin/env python

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

import roslib
roslib.load_manifest('arbotix')
import rospy

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from arbotix.arbotix import ArbotiX     # does this look ridiculous to anyone else?
from arbotix.srv import *
from arbotix.ax12 import *

from math import sin,cos,pi,radians
from datetime import datetime

device = None

###############################################################################
# Servo handling classes
class DynamixelServo():
    """ Class to handle services and updates for a single Dynamixel Servo, on 
        an ArbotiX robocontroller's AX-bus. """
    def __init__(self, index, params, device):
        self.id = index
        self.device = device
        self.angle = 0.0

        # process parameters
        self.params = { 
            'neutral':512, 
            'rad_per_tick': radians(300.0)/1024, 
            'max': radians(150),
            'min': radians(-150),
            'invert': False,
            'max_speed': radians(50)
        }
        for key in params.keys():
            if key=='invert':
                if int(params[key]) > 0:
                    self.params[key] = True
            elif key=='max_speed' or key=='max' or key=='min':
                self.params[key] = radians((params[key]))
            elif key=='range':
                self.params['rad_per_tick'] = radians(float(params[key]))/1024
            else:
                try:
                    self.params[key] = float(params[key])
                except:
                    self.params[key] = params[key]        
        self.name = self.params['name']

        # some callbacks
        self.srvRead = rospy.Service(self.name+'_getangle',GetAngle, self.__getAngle)
        self.srvMoving = rospy.Service(self.name+'_ismoving',IsMoving, self.__isMoving)
        self.srvWrite = rospy.Service(self.name+'_setangle',SetAngle, self.__setAngle)
        self.srvRelax = rospy.Service(self.name+'_relax',Relax, self.__relax)

    def __setAngle(self, req):
        ang = req.angle
        vel = req.velocity

        if vel == None:
            vel = self.params['max_speed']

        if vel > self.params['max_speed']:        
            vel = self.params['max_speed']

        if ang > self.params['max'] or ang < self.params['min']:
            rospy.logerr("Servo "+self.params['name']+": angle out of range ("+str(ang)+")")            
            return SetAngleResponse()
        
        rpm = vel / (2 * pi) * 60.0
        self.device.setSpeed(self.id, int(round( rpm / 0.111 )))

        if self.params['invert']:
            ang = ang * -1.0
        ticks = int(round( ang / self.params['rad_per_tick'] ))
        ticks += int(self.params['neutral'])
        self.device.setPosition(self.id, ticks)

        #if blocking == True:
        #    while(self.is_moving()):
        #        continue
        return SetAngleResponse()        

    def __getAngle( self, request ):
        """ ROS service to get angle of servo (in radians) """
        return GetAngleResponse( self.update() )

    def __isMoving( self, request ):
        status = self.device.read(self.id, P_MOVING, 1)[0]
        return IsMovingResponse( int(status) )

    def __relax( self, request ):
        """ Turn off servo torque, so that it is pose-able. """
        self.device.disableTorque(self.id)
        return RelaxResponse()

    def update(self, pos=None):
        """ Find angle in radians """
        if pos == None:
            pos = self.device.getPosition(self.id)
        if pos != -1:
            ang = (pos - self.params['neutral']) * self.params['rad_per_tick']
            if self.params['invert']:
                ang = ang * -1.0
            self.angle = ang
        return self.angle

class HobbyServo():
    """ Class to handle services and updates for a single Hobby Servo, connected to 
        an ArbotiX robocontroller. """
    def __init__(self, index, params, device):
        self.id = index
        self.device = device
        self.angle = 0.0

        # process parameters
        self.params = { 
            'neutral':1500, 
            'rad_per_tick': radians(180.0)/2000,       # 180 degrees over 500-2500ms 
            'max': radians(90),
            'min': radians(-90),
            'invert': False
        }
        for key in params.keys():
            if key=='invert':
                if int(params[key]) > 0:
                    self.params[key] = True
            elif key=='max' or key=='min':
                self.params[key] = radians(int(params[key]))
            else:
                try:
                    self.params[key] = float(params[key])
                except:
                    self.params[key] = params[key]
        self.name = self.params['name']

        # a callback
        self.srvWrite = rospy.Service(self.name+'_setangle',SetAngle, self.__setAngle)

    def __setAngle(self, req):
        """ Set position to angle, in radians. """
        ang = req.angle

        if ang > self.params['max'] or ang < self.params['min']:
            rospy.logerr("Servo "+self.params['name']+": angle out of range ("+str(ang)+")")            
            return SetAngleResponse()
        
        self.angle = ang    # store it for joint state updates
        if self.params['invert']:
            ang = ang * -1.0
        ticks = int(round( ang / self.params['rad_per_tick'] ))
        ticks += self.params['neutral']
        rospy.loginfo("Servo "+self.params['name']+": set to "+str(ticks))
        self.device.setServo(self.id, ticks)
        return SetAngleResponse()        

    def update(self):
        """ Find angle in radians """
        return self.angle

class ArbotiXBase():
    """ Class to handle services, odometry feedback, etc, for a mobile base, 
        powered by an ArbotiX RoboController. """
    def __init__(self, device, ticks_meter, base_width):
        self.device = device        # handle for robocontroller
        self.ticks_meter = ticks_meter
        self.base_width = base_width
        self.ticks_rad = (base_width*ticks_meter) / (2.0*pi)
        self.enc_left = 0           # encoder readings
        self.enc_right = 0
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        self.then = datetime.now()  # time for determining dx/dy
        rospy.Subscriber("cmd_vel", Twist, self.twist)

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

    #def turn(self,req):
    #    if (req.clear):
    #       self.create.clear()
    #    self.create.turn(req.turn)
    #    return TurnResponse(True)

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
            self.device.stopBase()
        else:
            self.device.moveBase(int(l),int(r),0,0)

        #if (x == 0):
        #    th = th*180/pi
        #    speed = (8*pi*th)/9
        #    self.create.left(int(speed))
        #elif (th == 0):
        #    x = int(x)
        #    self.create.tank(x,x)
        #else:
        #    self.create.forwardTurn(int(x),int(x/th))
        
def get_digital_callback( req ):
    return DigitalResponse( device.getDigital(req.pin) )

def get_analog_callback( req ):    
    return AnalogResponse( device.getAnalog(req.pin) )

#def set_digital_callback( req ):
#    pass

if __name__ == "__main__":
    rospy.init_node('arbotix')
    # short wait so that we are fully initialized
    rospy.sleep(0.5)

    # load configurations    
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = int(rospy.get_param("~baud", "38400"))
    rospy.loginfo("Starting ArbotiX-ROS on port "+port)
    dynamixels = rospy.get_param("~dynamixels", dict())
    servos = rospy.get_param("~servos", dict())
    grounding = rospy.get_param("~grounding","world")
    ticks_meter = float(rospy.get_param("~ticks_meter", "26154"))
    base_width = float(rospy.get_param("~base_width", "0.144"))
    do_sync = bool(rospy.get_param("~sync","True"))

    # start an arbotix driver
    device = ArbotiX(port, baud)
    
    # initialize servos and state publishing
    dynamixel_servos = dict()
    sync_servos = list()
    for index in dynamixels.keys():
        dynamixel_servos[int(index)] = DynamixelServo(int(index), dynamixels[index], device) 
        sync_servos.append(int(index))    

    hobby_servos = list()
    for index in servos.keys():
        hobby_servos.append( HobbyServo(int(index), servos[index], device) )
    
    jointStatePub = rospy.Publisher('joint_states', JointState)
    
    # digital/analog IO
    #rospy.Service('GetDigital',Digital,get_digital_callback)   
    #rospy.Service('GetAnalog',Analog,get_analog_callback)
    #rospy.Service('SetDigital',Digital,set_digital_callback)

    # listen for movement commands, send to robot
    #base = ArbotiXBase(device, ticks_meter, base_width)
    
    # publish joint states (everything else is a service callback)
    r = rospy.Rate(int(rospy.get_param("~rate",10)))
    while not rospy.is_shutdown():
        msg = JointState()
        msg.name = list()
        msg.position = list()
        msg.velocity = list()
        msg.effort = list()

        #for servo in dynamixel_servos + hobby_servos:
        #    msg.name.append(servo.name)
        #    msg.position.append(servo.update())  
    
        if do_sync: 
            val = device.syncRead(sync_servos, P_PRESENT_POSITION_L, 2)
            i = 0        
            for servo in sync_servos:
                msg.name.append(dynamixel_servos[servo].name)
                msg.position.append(dynamixel_servos[servo].update( val[i]+(val[i+1]<<8) ))
                i = i + 2
        else:
            for servo in dynamixel_servos.values():
                msg.name.append(servo.name)
                msg.position.append(servo.update())  
            
        for servo in hobby_servos:
            msg.name.append(servo.name)
            msg.position.append(servo.update())  

        msg.header.stamp = rospy.Time.now()
        jointStatePub.publish(msg)                  
        r.sleep()

