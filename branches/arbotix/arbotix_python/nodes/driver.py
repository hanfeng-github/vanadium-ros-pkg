#!/usr/bin/env python

"""
  ArbotiX ROS Node: serial connection to an ArbotiX board w/ PyPose/NUKE/ROS
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
from trajectory_msgs.msg import *
from arbotix_msgs.msg import *
from arbotix_msgs.srv import *

from arbotix_python.arbotix import ArbotiX # does this look ridiculous to anyone else?
from arbotix_python.ax12 import *
from arbotix_python.base import *
from arbotix_python.pml import *

from math import sin,cos,pi,radians

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
        self.sync = rospy.get_param(n+"sync",device.use_sync)

        self.angle = 0.0               # current position
        self.desired = 0.0             # desired position
        self.velocity = 0.0            # current velocity
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', JointTrajectoryPoint, self.commandCb)
        rospy.Subscriber(name+'/simple_command', Float64, self.simple_commandCb)
        self.srvRelax = rospy.Service(name+'relax',Relax, self.relaxCb)

    def relaxCb(self, req):
        """ Turn off servo torque, so that it is pose-able. """
        self.device.disableTorque(self.id)
        return RelaxResponse()

    def commandCb(self, req):
        # TODO: make this more advanced
        try:
            self.desired = req.positions[0]
            self.velocity = req.velocities[0]
        except:
            pass

    def simple_commandCb(self, req):
        self.desired = req.data
        print self.desired
        
    def update(self, value):
        """ Update angle in radians by reading from servo, or
            by using pos passed in from a sync read.  """
        if value < 0:
            # read servo
            if self.sync:
                value = self.device.getPosition(self.id)
        if value != -1:
            if self.invert:
                self.angle = -1.0 * (value - self.neutral) * self.rad_per_tick
            else:
                self.angle = (value - self.neutral) * self.rad_per_tick

    def interpolate(self, frame):
        """ Get the new position to move to, in ticks. """
        return self.neutral + (self.desired/self.rad_per_tick)

#    def setAngle(self, ang):
#        if ang > self.max_angle or ang < self.min_angle:
#            rospy.logerr("Servo "+self.name+": angle out of range ("+str(ang)+"). Limits are: " + str(self.min_angle) + "," + str(self.max_angle))  
#            return 
#        self.angle = ang    # store it for joint state updates
#        if self.invert:
#            ang = ang * -1.0
#        ticks = int(round( ang / self.rad_per_tick ))
#        ticks += int(self.neutral)
#        self.device.setPosition(self.id, ticks)

class HobbyServo(Servo):
    """ Class to handle services and updates for a single Hobby Servo, connected to 
        an ArbotiX robocontroller. """
    def __init__(self, name, device):
        self.name = name
        self.device = device           # ArbotiX instance
        n = "~servos/"+name+"/"

        # TODO: load URDF specs

        self.id = int(rospy.get_param(n+"id"))
        self.neutral = rospy.get_param(n+"neutral",1500) # might be adjusted for crappy servos
        self.ticks = rospy.get_param(n+"ticks",2000)
        self.rad_per_tick = radians(rospy.get_param(n+"range",180.0))/self.ticks

        self.max_angle = radians(rospy.get_param(n+"max_angle",90))
        self.min_angle = radians(rospy.get_param(n+"min_angle",-90))

        self.invert = rospy.get_param(n+"invert",False)
        self.sync = rospy.get_param(n+"sync",device.use_sync)

        self.angle = 0.0               # current position
        self.desired = 0.0             # desired position
        
        # ROS interfaces
        #rospy.Subscriber(name+'/command', JointTrajectoryPoint, self.commandCb)
        rospy.Subscriber(name+'/simple_command', Float64, self.simple_commandCb)   

    def simple_commandCb(self, req):
        self.desired = req.data

    def update(self, value):
        """ For model compliance only -- we can't read the value of a hobby servo. """
        pass

    def interpolate(self, frame):
        """ Get the new position to move to, in ticks. """
        return self.neutral


###############################################################################
# IO Infrastructure

class DigitalServo:
    def __init__(self, name, device):
        self.device = device
        self.value = 0
        self.direction = 0
        self.pin = rospy.get_param('~digital_servos/'+name+'/pin')
        self.throttle = rospy.get_param('~digital_servos/'+name+'/throttle')
        rospy.Subscriber('~'+name, Digital, self.stateCb)
    def stateCb(self, msg):
        self.value = msg.value
        self.direction = msg.direction
    def update(self):
        self.device.setDigital(self.pin, self.value, self.direction)

class DigitalSensor:
    def __init__(self, name, device):
        self.device = device
        self.pin = rospy.get_param('~digital_sensors/'+name+'/pin')
        self.throttle = rospy.get_param('~digital_sensors/'+name+'/throttle')
        self.pub = rospy.Publisher('~'+name, Digital)
    def update(self):
        msg = Digital()
        msg.value = self.device.getDigital(self.pin)
        self.pub.publish(msg)

class AnalogSensor: 
    def __init__(self, name, device):
        self.device = device
        self.pin = rospy.get_param('~analog_sensors/'+name+'/pin')
        self.throttle = rospy.get_param('~analog_sensors/'+name+'/throttle')
        self.pub = rospy.Publisher('~'+name, Analog)
    def update(self):
        msg = Digital()
        msg.value = self.device.getAnalog(self.pin)
        self.pub.publish(msg)


###############################################################################
# Main ROS interface
class ArbotiX_ROS(ArbotiX):
    
    def __init__(self):
        rospy.init_node('arbotix')
        # load configurations    
        port = rospy.get_param("~port", "/dev/ttyUSB0")                     
        baud = int(rospy.get_param("~baud", "115200"))     
        self.rate = int(rospy.get_param("~rate", "100"))
        self.throttle_r = int(rospy.get_param("~read/throttle", "10")) # throttle rate for read
        self.throttle_w = int(rospy.get_param("~write/throttle", "10")) # throttle rate for write
        self.use_sync  = rospy.get_param("~use_sync",True) # use sync read?

        # start an arbotix driver
        ArbotiX.__init__(self, port, baud)        
        rospy.loginfo("Started ArbotiX connection on port "+port)
        
        # initialize dynamixel & hobby servos
        dynamixels = rospy.get_param("~dynamixels", dict())
        self.dynamixels = dict()
        for name in dynamixels.keys():
            self.dynamixels[name] = Servo(name,self)
        hobbyservos = rospy.get_param("~servos", dict())
        self.servos = dict()
        for name in hobbyservos.keys():
            self.servos[name] = HobbyServo(name, self)

        # joint state publication
        self.jointStatePub = rospy.Publisher('joint_states', JointState)

        # initialize digital/analog IO streams
        self.io = dict()
        for v,t in {"digital_servos":DigitalServo,"digital_sensors":DigitalSensor,"analog_sensors":AnalogSensor}.items():
            temp = rospy.get_param(v,dict())
            for name in temp.keys():
                self.io[name] = t(name,self)
        
        # setup a base controller
        self.use_base = False
        if rospy.has_param("~base"):
            self.use_base = True
            self.base = base_controller(self)
            self.base.startup()

        # setup a pml  
        self.use_pml = False
        if rospy.has_param("~pml"):
            self.use_pml = True
            self.pml = pml(self)
            self.pml.startup()

        r = rospy.Rate(self.rate)
        f = 0  # frame ID
        # main loop -- do all the read/write here
        while not rospy.is_shutdown():
    
            # update servo positions (via sync_write)
            if f%self.throttle_w == 0:
                syncpkt = list()
                for servo in self.dynamixels.values():
                    v = int(servo.interpolate(self.rate/self.throttle_w))
                    syncpkt.append([servo.id,v%256,v>>8])        
                self.syncWrite(P_GOAL_POSITION_L,syncpkt)

            # update base
            if self.use_base and f%self.base.throttle == 0:
                self.base.update()

            # update pml
            if self.use_pml and f%self.pml.throttle == 0:
                self.pml.update()

            # update io
            for s in self.io.values():
                if f%s.throttle == 0:
                    s.update()

            # publish joint states
            if f%self.throttle_r == 0:
                # TODO: add torque/heat recovery
                #   a.write(id,P_TORQUE_LIMIT_L,[255,3])
                try:
                    if self.use_sync:
                        # arbotix/servostik/wifi board sync_read
                        synclist = list()
                        for servo in self.dynamixels.values():
                            if servo.sync:
                                synclist.append(servo.id)
                        if len(synclist) > 0:
                            val = self.syncRead(synclist, P_PRESENT_POSITION_L, 2)
                            if val != None: 
                                for servo in self.dynamixels.values():
                                    try:
                                        i = synclist.index(servo.id)
                                        servo.update(val[i]+(val[i+1]<<8))
                                    except:
                                        continue # not a sync readable servo
                    else:
                        # direct connection, or other hardware with no sync_read capability
                        for servo in self.dynamixels.values():
                            servo.update(-1)
                except:
                    rospy.loginfo("Error in filling joint_states message")             
                        
                # publish joint states         
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = list()
                msg.position = list()
                msg.velocity = list()
                for servo in self.dynamixels.values() + self.servos.values():
                    msg.name.append(servo.name)
                    msg.position.append(servo.angle)
                    msg.velocity.append(servo.velocity)
                self.jointStatePub.publish(msg)   

            f += 1
            r.sleep()      
        # do shutdown
        if self.use_base:
            self.base.shutdown()
        if self.use_pml:
            self.pml.shutdown()

if __name__ == "__main__":
    a = ArbotiX_ROS()

