#!/usr/bin/env python

"""
  ArbotiX Node: serial connection to an ArbotiX board w/ PyPose/NUKE/ROS
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

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

import roslib; roslib.load_manifest('arbotix_python')
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from diagnostic_msgs.msg import *
from arbotix_msgs.msg import *
from arbotix_msgs.srv import *

from arbotix_python.arbotix import ArbotiX # does this look ridiculous to anyone else?
from arbotix_python.ax12 import *
from arbotix_python.base import *
from arbotix_python.pml import *

from math import radians

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
        self.voltage = 0.0
        self.temperature = 0.0
        self.last = rospy.Time.now()
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)
        rospy.Service(name+'/relax', Relax, self.relaxCb)

    def relaxCb(self, req):
        """ Turn off servo torque, so that it is pose-able. """
        self.device.disableTorque(self.id)
        self.relaxed = True
        return RelaxResponse()

    def commandCb(self, req):
        if self.desired != req.data:
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
            self.velocity = (self.angle - last_angle)/((t - self.last).to_nsec()/1000000000.0)
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
            if self.invert:
                return self.neutral - (self.last_cmd/self.rad_per_tick)
            else:
                return self.neutral + (self.last_cmd/self.rad_per_tick)
        else:
            return None


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
        self.readable = False          # can't read a hobby servo!

        self.dirty = False             # newly updated position?
        self.angle = 0.0               # current position
        self.velocity = 0.0            # this currently doesn't provide info for hobby servos
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)

    def commandCb(self, req):
        """ Callback to set position to angle, in radians. """
        self.dirty = True
        self.angle = req.data

    def update(self, value):
        """ If dirty, update value of servo at device. """
        if self.dirty:
            # test limits
            if self.angle < self.min_angle:
                self.angle = self.min_angle
            if self.angle > self.max_angle:
                self.angle = self.max_angle
            # send update to hobby servo
            ang = self.angle
            if self.invert:
                ang = ang * -1.0
            ticks = int(round( ang / self.rad_per_tick ))
            self.device.setServo(self.id, ticks)
            self.dirty = False


###############################################################################
# IO Infrastructure

class DigitalServo:
    def __init__(self, name, pin, value, rate, device):
        self.device = device
        self.value = value
        self.direction = 0
        self.pin = pin
        self.throttle = device.rate/rate
        self.device.setDigital(self.pin, self.value, self.direction)
        rospy.Subscriber('~'+name, Digital, self.stateCb)
    def stateCb(self, msg):
        self.value = msg.value
        self.direction = msg.direction
    def update(self):
        self.device.setDigital(self.pin, self.value, self.direction)

class DigitalSensor:
    def __init__(self, name, pin, value, rate, device):
        self.device = device
        self.pin = pin
        self.throttle = device.rate/rate
        self.device.setDigital(pin, value, 0)
        self.pub = rospy.Publisher('~'+name, Digital)
    def update(self):
        msg = Digital()
        msg.header.stamp = rospy.Time.now()
        msg.value = self.device.getDigital(self.pin)
        self.pub.publish(msg)

class AnalogSensor: 
    def __init__(self, name, pin, value, rate, device):
        self.device = device
        self.pin = pin
        self.throttle = device.rate/rate
        self.device.setDigital(pin, value, 0)
        self.pub = rospy.Publisher('~'+name, Analog)
    def update(self):
        msg = Analog()
        msg.header.stamp = rospy.Time.now()
        msg.value = self.device.getAnalog(self.pin)
        self.pub.publish(msg)


###############################################################################
# Main ROS interface
class ArbotixROS(ArbotiX):
    
    def __init__(self):
        rospy.init_node('arbotix')
        # load configurations    
        port = rospy.get_param("~port", "/dev/ttyUSB0")                     
        baud = int(rospy.get_param("~baud", "115200"))     
        self.rate = int(rospy.get_param("~rate", 100))
        self.throttle_r = int(self.rate/rospy.get_param("~read_rate", 10)) # throttle rate for read
        self.throttle_w = int(self.rate/rospy.get_param("~write_rate", 10)) # throttle rate for write
        self.throttle_d = int(self.rate/rospy.get_param("~diagnostic_rate", 1))
        self.diagnostic_update = int(rospy.get_param("~diagnostic_update", 5))  # how often to update temperature/voltage 
        self.use_sync_read = rospy.get_param("~sync_read",True)      # use sync read?
        self.use_sync_write = rospy.get_param("~sync_write",True)    # use sync write?

        # start an arbotix driver
        ArbotiX.__init__(self, port, baud)        
        rospy.sleep(1.0)
        rospy.loginfo("Started ArbotiX connection on port " + port)
        
        # initialize dynamixel & hobby servos
        dynamixels = rospy.get_param("~dynamixels", dict())
        self.dynamixels = dict()
        for name in dynamixels.keys():
            self.dynamixels[name] = Servo(name,self)
        hobbyservos = rospy.get_param("~servos", dict())
        self.servos = dict()
        for name in hobbyservos.keys():
            self.servos[name] = HobbyServo(name, self)

        # publishers, subscribers and services
        self.jointStatePub = rospy.Publisher('joint_states', JointState)
        self.diagnosticPub = rospy.Publisher('diagnostics', DiagnosticArray)
        rospy.Service('~SetupAnalogIn',SetupChannel, self.analogInCb)
        rospy.Service('~SetupDigitalIn',SetupChannel, self.digitalInCb)
        rospy.Service('~SetupDigitalOut',SetupChannel, self.digitalOutCb)

        # initialize digital/analog IO streams
        self.io = dict()
        for v,t in {"digital_servos":DigitalServo,"digital_sensors":DigitalSensor,"analog_sensors":AnalogSensor}.items():
            temp = rospy.get_param("~"+v,dict())
            for name in temp.keys():
                pin = rospy.get_param('~'+v+'/'+name+'/pin',1)
                value = rospy.get_param('~'+v+'/'+name+'/value',0)
                rate = rospy.get_param('~'+v+'/'+name+'/rate',10)
                self.io[name] = t(name, pin, value, rate, self)
        
        # setup a base controller
        self.use_base = False
        if rospy.has_param("~base"):
            self.use_base = True
            self.base = BaseController(self)
            self.base.startup()

        # setup a pml  
        self.use_pml = False
        if rospy.has_param("~pml"):
            self.use_pml = True
            self.pml = pml(self)
            self.pml.startup()

        r = rospy.Rate(self.rate)
        f = 0  # frame ID
        d = 0  # diagnostic count
        # main loop -- do all the read/write here
        while not rospy.is_shutdown():
    
            # diagnostics            
            if f%self.throttle_d == 0:
                if d == 0:
                    # update status of servos
                    if self.use_sync_read:
                        # arbotix/servostik/wifi board sync_read
                        synclist = list()
                        for servo in self.dynamixels.values():
                            if servo.readable:
                                synclist.append(servo.id)
                            else:
                                servo.update(-1)
                        if len(synclist) > 0:
                            val = self.syncRead(synclist, P_PRESENT_VOLTAGE, 2)
                            if val: 
                                for servo in self.dynamixels.values():
                                    try:
                                        i = synclist.index(servo.id)*2
                                        servo.voltage = val[i]/10.0
                                        servo.temperature = val[i+1]
                                    except:
                                        # not a readable servo
                                        continue 
                    else:
                        # direct connection, or other hardware with no sync_read capability
                        for servo in self.dynamixels.values():
                            if servo.readable:
                                try:
                                    val = self.read(servo.id, P_PRESENT_VOLTAGE, 2)
                                    servo.voltage = val[0]
                                    servo.temperature = val[1]
                                except:
                                    pass
                # publish diagnostics data
                msg = DiagnosticArray()
                msg.header.stamp = rospy.Time.now()
                for servo in self.dynamixels.values():
                    stat = DiagnosticStatus()
                    stat.name = "Joint " + servo.name
                    stat.level = DiagnosticStatus.OK
                    stat.message = "OK"
                    stat.values.append(KeyValue("Position", str(servo.angle)))
                    stat.values.append(KeyValue("Temperature", str(servo.temperature)))
                    if servo.temperature > 60:
                        stat.level = DiagnosticStatus.ERROR
                        stat.message = "OVERHEATED"
                    elif servo.temperature > 40:
                        stat.level = DiagnosticStatus.WARN
                        stat.message = "VERY HOT"
                    stat.values.append(KeyValue("Voltage", str(servo.voltage)))
                    if servo.relaxed:
                        stat.values.append(KeyValue("Torque", "OFF"))
                    else:
                        stat.values.append(KeyValue("Torque", "ON"))
                    msg.status.append(stat)
                if self.use_base:
                    stat = DiagnosticStatus()
                    stat.name = "Encoders"
                    stat.level = DiagnosticStatus.OK
                    stat.message = "OK"
                    stat.values.append(KeyValue("Left", str(self.base.enc_left)))
                    stat.values.append(KeyValue("Right", str(self.base.enc_right)))
                    msg.status.append(stat)
                self.diagnosticPub.publish(msg)
                # update diagnostic counter
                d = (d+1)%self.diagnostic_update
    
            # update servo positions (via sync_write)
            if f%self.throttle_w == 0:
                if self.use_sync_write:
                    syncpkt = list()
                    for servo in self.dynamixels.values():
                        v = servo.interpolate(self.rate/self.throttle_w)
                        if v != None:   # if was dirty
                            syncpkt.append([servo.id,int(v)%256,int(v)>>8])  
                    if len(syncpkt) > 0:      
                        self.syncWrite(P_GOAL_POSITION_L,syncpkt)
                else:
                    for servo in self.dynamixels.values():
                        v = servo.interpolate(self.rate/self.throttle_w)
                        if v != None:   # if was dirty      
                            self.setPosition(servo.id, int(v))

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
                    if self.use_sync_read:
                        # arbotix/servostik/wifi board sync_read
                        synclist = list()
                        for servo in self.dynamixels.values():
                            if servo.readable:
                                synclist.append(servo.id)
                            else:
                                servo.update(-1)
                        if len(synclist) > 0:
                            val = self.syncRead(synclist, P_PRESENT_POSITION_L, 2)
                            if val: 
                                for servo in self.dynamixels.values():
                                    try:
                                        i = synclist.index(servo.id)*2
                                        servo.update(val[i]+(val[i+1]<<8))
                                    except:
                                        # not a readable servo
                                        continue 
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
                if self.use_base:
                    msg.name += self.base.names
                    msg.position += self.base.positions
                    msg.velocity += self.base.velocities
                self.jointStatePub.publish(msg)

            f += 1
            r.sleep()      
        # do shutdown
        if self.use_base:
            self.base.shutdown()
        if self.use_pml:
            self.pml.shutdown()

    # IO Callbacks
    def analogInCb(self, req):
        # TODO: Add check, only 1 service per pin
        self.io[req.topic_name] = AnalogSensor(req.topic_name, req.pin, req.value, req.rate, self) 
        return SetupChannelResponse()
    def digitalInCb(self, req):
        self.io[req.topic_name] = DigitalSensor(req.topic_name, req.pin, req.value, req.rate, self) 
        return SetupChannelResponse()
    def digitalOutCb(self, req):
        self.io[req.topic_name] = DigitalServo(req.topic_name, req.pin, req.value, req.rate, self) 
        return SetupChannelResponse()


if __name__ == "__main__":
    a = ArbotixROS()

