#!/usr/bin/env python

"""
  servos.py: helper functions for servo interactions
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

import rospy

from math import pi, radians
import xml.dom.minidom

from std_msgs.msg import Float64
from arbotix_msgs.srv import Relax

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
        if self.desired != req.data or self.relaxed:
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


def getServosFromURDF():
    """ Get servo parameters from URDF. """
    try:
        description = rospy.get_param("robot_description")
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        joints = {}
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                  continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                joints[name] = joint
        return joints
    except:
        rospy.loginfo('No URDF defined, proceeding with defaults')
        return dict()


def getServoLimits(name, joint_defaults, default_min=-150, default_max=150):
    """ Get limits of servo, from YAML, then URDF, then defaults if neither is defined. """
    min_angle = radians(default_min)
    max_angle = radians(default_max)
    
    try: 
        min_angle = joint_defaults[name]['min']
    except:
        pass
    try: 
        min_angle = radians(rospy.get_param("/arbotix/dynamixels/"+name+"/min_angle"))
    except:
        pass

    try: 
        max_angle = joint_defaults[name]['max']
    except:
        pass
    try: 
        max_angle = radians(rospy.get_param("/arbotix/dynamixels/"+name+"/max_angle"))
    except:
        pass

    return (min_angle, max_angle)

