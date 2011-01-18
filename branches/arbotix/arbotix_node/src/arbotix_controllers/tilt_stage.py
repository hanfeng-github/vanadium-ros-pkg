#!/usr/bin/env python

"""
  tilt_stage.py - A periodic laser tilt stage
  Copyright (c) 2010 Vanadium Labs LLC.  All right reserved.

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

import rospy
from threading import Thread
from math import radians

class tilt_stage(Thread):
    """ A class for tilting a laser stage. """

    def __init__(self, device, name):
        Thread.__init__ (self)

        # handle for robocontroller
        self.device = device
        self.name = name

        # parameters
        self.rate = rospy.get_param("~controllers/"+name+"/rate",10) 
        self.joint = rospy.get_param("~controllers/"+name+"/joint","laser_tilt_mount_joint")
        self.lower_bound = rospy.get_param("~controllers/"+name+"/lower_bound",400) - 512
        self.upper_bound = rospy.get_param("~controllers/"+name+"/upper_bound",600) - 512
        self.step_value = rospy.get_param("~controllers/"+name+"/step",10)

        rospy.loginfo("Started tilt_stage controller '"+name+"' using servo: " + str(self.joint))

    def run(self):
        r = rospy.Rate(self.rate)
        self.step = -self.step_value
        pos = self.step
        while not rospy.is_shutdown():
            if self.step > 0:
                pos += self.step
                if pos > self.upper_bound:
                    pos = self.upper_bound
                    self.step = -self.step_value
            else:
                pos += self.step         
                if pos < self.lower_bound:
                    pos = self.lower_bound
                    self.step = self.step_value
            self.device.servos[self.joint].setAngle( (pos)*(radians(300.0)/1024) )
            #print "Move laser to " + str(pos)
            r.sleep()

