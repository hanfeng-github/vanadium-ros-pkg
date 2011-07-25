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
from arbotix_msgs.msg import *
from arbotix_msgs.srv import *

from arbotix_python.arbotix import ArbotiX
from arbotix_python.diff_controller import DiffController
from arbotix_python.diagnostics import DiagnosticsPublisher
from arbotix_python.ax12 import P_PRESENT_POSITION_L, P_GOAL_POSITION_L
from arbotix_python.io import *
from arbotix_python.servos import *


class JointStatePublisher:
    """ Class to handle publications of joint_states message. """

    def __init__(self, device):

        # handle for robocontroller
        self.device = device

        # parameters: throttle rate and geometry
        self.rate = rospy.get_param("~read_rate", 10.0)
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        # subscriber
        self.pub = rospy.Publisher('joint_states', JointState)

    def update(self):
        try:
            if self.device.use_sync_read:
                # arbotix/servostik/wifi board sync_read
                synclist = list()
                for servo in self.device.dynamixels.values():
                    if servo.readable:
                        synclist.append(servo.id)
                    else:
                        servo.update(-1)
                if len(synclist) > 0:
                    val = self.device.syncRead(synclist, P_PRESENT_POSITION_L, 2)
                    if val: 
                        for servo in self.device.dynamixels.values():
                            try:
                                i = synclist.index(servo.id)*2
                                servo.update(val[i]+(val[i+1]<<8))
                            except:
                                # not a readable servo
                                continue 
            else:
                # direct connection, or other hardware with no sync_read capability
                for servo in self.device.dynamixels.values():
                    servo.update(-1)
        except:
            rospy.loginfo("Error in filling joint_states message")  
            return           
                        
        # publish joint states         
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list()
        msg.position = list()
        msg.velocity = list()
        for servo in self.device.dynamixels.values() + self.device.servos.values():
            msg.name.append(servo.name)
            msg.position.append(servo.angle)
            msg.velocity.append(servo.velocity)
        if self.device.base:
            msg.name += self.device.base.joint_names
            msg.position += self.device.base.joint_positions
            msg.velocity += self.device.base.joint_velocities
        self.pub.publish(msg)


###############################################################################
# Main ROS interface
class ArbotixROS(ArbotiX):
    
    def __init__(self):
        # load configurations    
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = int(rospy.get_param("~baud", "115200"))

        self.rate = rospy.get_param("~rate", 50.0)
        self.t_delta = rospy.Duration(1.0/rospy.get_param("~write_rate", 10.0))
        self.t_next = rospy.Time.now() + self.t_delta

        self.use_sync_read = rospy.get_param("~sync_read",True)      # use sync read?
        self.use_sync_write = rospy.get_param("~sync_write",True)    # use sync write?

        # setup publishers
        self.diagnostics = DiagnosticsPublisher(self)
        self.joint_state_publisher = JointStatePublisher(self)

        # start an arbotix driver
        ArbotiX.__init__(self, port, baud)        
        rospy.sleep(1.0)
        rospy.loginfo("Started ArbotiX connection on port " + port + ".")
        
        # wait for arbotix to start up (especially after reset)
        if rospy.has_param("~base") or rospy.has_param("~digital_servos") or rospy.has_param("~digital_sensors") or rospy.has_param("~analog_sensors"):
            while self.getDigital(1) == -1:
                rospy.loginfo("Waiting for response...")
                rospy.sleep(0.25)
            rospy.loginfo("ArbotiX connected.")

        # TODO: initialize dynamixel & hobby servos
        dynamixels = rospy.get_param("~dynamixels", dict())
        self.dynamixels = dict()
        for name in dynamixels.keys():
            self.dynamixels[name] = Servo(name,self)
        hobbyservos = rospy.get_param("~servos", dict())
        self.servos = dict()
        for name in hobbyservos.keys():
            self.servos[name] = HobbyServo(name, self)

        # setup base controllers
        self.base = None
        if rospy.has_param("~base"):
            if rospy.has_param("~base/omni"):
                self.base = OmniController(self)
            else:
                self.base = DiffController(self)
            self.base.startup()

        # setup trajectory actions
#         TODO

        # services for io
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
        
        r = rospy.Rate(self.rate)

        # main loop -- do all the read/write here
        while not rospy.is_shutdown():
    
            # TODO: update controllers

            # update servo positions (via sync_write)
            self.update()
    
            # update base
            if self.base:
                self.base.update()

            # update io
            for io in self.io.values():
                io.update()

            # publish
            self.joint_state_publisher.update()
            self.diagnostics.update()

            r.sleep()

        # do shutdown
        if self.base:
            self.base.shutdown()


    def update(self):
        """ Write updated positions to servos. """
        if rospy.Time.now() > self.t_next:
            if self.use_sync_write:
                syncpkt = list()
                for servo in self.dynamixels.values():
                    v = servo.interpolate(self.t_delta.to_sec())
                    if v != None:   # if was dirty
                        syncpkt.append([servo.id,int(v)%256,int(v)>>8])  
                if len(syncpkt) > 0:      
                    self.syncWrite(P_GOAL_POSITION_L,syncpkt)
            else:
                for servo in self.dynamixels.values():
                    v = servo.interpolate(self.t_delta.to_sec())
                    if v != None:   # if was dirty      
                        self.setPosition(servo.id, int(v))
            self.t_next = rospy.Time.now() + self.t_delta

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
    rospy.init_node('arbotix')
    a = ArbotixROS()

