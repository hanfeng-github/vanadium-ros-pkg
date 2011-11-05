#!/usr/bin/env python

"""
  linear_controller.py - controller for a linear actuator with analog feedback
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

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

import rospy, actionlib

from joints import *
from controllers import *
from std_msgs.msg import Float64
from diagnostic_msgs.msg import *

class LinearJoint(Joint):
    def __init__(self, device, name):
        Joint.__init__(self, device, name)

        self.dirty = False
        self.position = 0.0                     # current position, as returned by feedback (meters)
        self.desired = 0.0                      # desired position (meters)
        self.velocity = 0.0                     # moving speed
        self.last = rospy.Time.now()

        # TODO: load these from URDF
        self.min = rospy.get_param('~controllers/'+name+'/min_position',0.0)
        self.max = rospy.get_param('~controllers/'+name+'/max_position',0.5)
        self.max_speed = rospy.get_param('~controllers/'+name+'/max_speed',0.05)

        # calibration data {reading: position} 
        self.cal = { 72: 0, 67: 0.0127,  64: 0.0255, 52: 0.0762, 44: 0.127, 38: 0.1778, 33: 0.2286, 30: 0.2794, 24: 0.4064, 20: 0.4953 }
        self.keys = sorted(self.cal.keys())

        rospy.Subscriber(name+'/command', Float64, self.commandCb)
        
    def interpolate(self, frame):
        """ Get new output: 1 = increase position, -1 is decrease position. """
        if self.dirty:
            cmd = self.desired - self.position
            if cmd > 0.01:
                return 1
            elif cmd < -0.01:
                return -1
            else:
                self.dirty = False
                return 0
        else:
            return None

    def setCurrentFeedback(self, reading):
        if reading >= self.keys[0] and reading <= self.keys[-1]:
            last_angle = self.position
            self.position = self.readingToPosition(reading)
            # update velocity estimate
            t = rospy.Time.now()
            self.velocity = (self.position - last_angle)/((t - self.last).to_nsec()/1000000000.0)
            self.last = t

    def setControlOutput(self, position):
        """ Set the position that controller is moving to. 
            Returns output value in raw_data format. """
        self.desired = position
        self.dirty = True
        return None # TODO
    
    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        if self.dirty:        
            msg.message = "Moving"
        else:
            msg.message = "OK"
        msg.values.append(KeyValue("Position", str(self.position)))
        return msg

    def commandCb(self, req):
        """ Float64 style command input. """
        if self.device.fake:
            self.position = req.data
        else:
            self.desired = req.data
            self.dirty = True

    def readingToPosition(self, reading):
        low = 0
        while reading > self.keys[low+1]:
            low += 1
        high = len(self.keys) - 1
        while reading < self.keys[high-1]:
            high += -1
        x = self.keys[high] - self.keys[low]
        y = self.cal[self.keys[high]] - self.cal[self.keys[low]]
        x1 = reading - self.keys[low]
        y1 = y * ( float(x1)/float(x) )
        #print reading, low, high, x, y, x1, self.cal[self.keys[high]] + y1, self.keys                         
        return self.cal[self.keys[low]] + y1


class LinearController(Controller):
    """ A controller for a linear actuator. """

    def __init__(self, device, name):
        Controller.__init__(self, device, name)

        self.a = rospy.get_param('~controllers/'+name+'/motor_a',13)
        self.b = rospy.get_param('~controllers/'+name+'/motor_b',14)
        self.p = rospy.get_param('~controllers/'+name+'/motor_pwm',15)
        print self.a, self.b, self.p
        self.analog = rospy.get_param('~controllers/'+name+'/feedback',0)
        self.last = 0

        self.delta = rospy.Duration(1.0/rospy.get_param('~controllers/'+name+'/rate', 10.0))
        self.next = rospy.Time.now() + self.delta

        self.joint = LinearJoint(device, name)
        device.joints[name] = self.joint

        # ROS interfaces
        rospy.loginfo("Started LinearController ("+self.name+").")

    def startup(self):
        self.joint.setCurrentFeedback(self.device.getAnalog(self.analog))

    def update(self):
        now = rospy.Time.now()
        if now > self.next:
            # read current position
            if self.joint.dirty:
                self.joint.setCurrentFeedback(self.device.getAnalog(self.analog))
            # update movement
            output = self.joint.interpolate(1.0/self.delta.to_sec())
            if self.last != output: 
                self.last = output
                #print self.device.getAnalog(self.analog), self.joint.position, self.joint.dirty, self.joint.desired, output
                if output == 1:
                    self.device.setDigital(self.a, 0); self.device.setDigital(self.b, 1); # up
                    self.device.setDigital(self.p, 1)
                    print "up"
                elif output == -1:
                    self.device.setDigital(self.a, 1); self.device.setDigital(self.b, 0); # down
                    self.device.setDigital(self.p, 1)
                    print "down"
                elif output == 0:
                    self.device.setDigital(self.p, 0)
                    print "stop"
            self.next = now + self.delta
    
    def shutdown(self):
        self.device.setDigital(self.p, 0)

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        return msg

