#!/usr/bin/env python

"""
  pml.py - a Poor Man's Scanning Laser
  Copyright (c) 2008-2010 Vanadium Labs LLC.  All right reserved.

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
from sensor_msgs.msg import LaserScan
from std_srvs.srv import *

class pml(Thread):
    """ Laser scan sensor interface. """

    def __init__(self, device, name):
        Thread.__init__ (self)

        # handle for robocontroller
        self.device = device
        self.name = name

        # parameters
        self.rate = rospy.get_param("~sensors/"+name+"/rate",1.0)
        self.frame_id = rospy.get_param("~sensors/"+name+"/frame","base_laser") 
        self.servo_id = rospy.get_param("~sensors/"+name+"/id",200)
        
        # annoyingly loud, allow servo panning to be turned on/off
        self.enable = False
        rospy.Service('EnablePML',Empty,self.enable_callback)   
        rospy.Service('DisablePML',Empty,self.disable_callback)

        # publisher
        self.scanPub = rospy.Publisher('base_scan', LaserScan)

        rospy.loginfo("Started pml sensor '"+name+"' using servo: " + str(self.servo_id))

    def run(self):
        # set PML servo ID
        self.device.write(253, self.device.PML_SERVO, [self.servo_id, 0])
        # run
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.enable:
                # get ranges
                v = self.device.read(253, self.device.PML_BASE, 60)
                ranges = list()
                for i in range(30):
                    k = v[i*2] + (v[i*2+1]<<8)
                    if k > 100:
                        # TODO: add other sensor abilities
                        ranges.append( (497.0/(k-56)) )
                    else:
                        ranges.append(0.0)
                # now post laser scan
                scan = LaserScan()
                scan.header.stamp = rospy.Time.now()        
                scan.header.frame_id = self.frame_id
                scan.angle_min = -1.57
                scan.angle_max = 1.57
                scan.angle_increment = 0.108275862
                scan.scan_time = self.rate
                scan.range_min = 0.5
                scan.range_max = 6.0
                scan.ranges = ranges    
                self.scanPub.publish(scan)
            r.sleep()

    def enable_callback(self, req):
        self.device.enablePML(True)
        self.enable = True
        return EmptyResponse()
    
    def disable_callback(self, req):
        self.enable = False
        self.device.enablePML(False)
        return EmptyResponse()

