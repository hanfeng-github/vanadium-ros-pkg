#!/usr/bin/env python

"""
  pml.py - the Planer Meta-Laser (formerly Poor Man's Scanning Laser)
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

    PML_SERVO = 80      # servo ID
    PML_SENSOR = 81
    PML_MIN_L = 82      # minimum position (in ticks)
    PML_STEP = 84       # step, in ticks
    PML_STEPS = 85      # number of steps in a scan (max 30)
    PML_ENABLE = 86     # scan or no?    

    PML_DIR = 88        # direction of read scan
    PML_TIME_L = 89     # time offset from first reading
    PML_BASE = 91       # base of readings array, array goes to index 150
    
    UP_SCAN = 0         # Right to left...?
    DN_SCAN = 1

    def __init__(self, device, name):
        Thread.__init__ (self)

        # handle for robocontroller
        self.device = device
        self.name = name

        # parameters
        self.rate = rospy.get_param("~sensors/"+name+"/rate",1.0)
        self.frame_id = rospy.get_param("~sensors/"+name+"/frame","base_laser") 
        self.servo_id = rospy.get_param("~sensors/"+name+"/servo_id",200) 
        self.sensor_id = rospy.get_param("~sensors/"+name+"/sensor_id",0)
        self.pml_id = rospy.get_param("~sensors/"+name+"/pml_id",0)

        self.step_start = rospy.get_param("~sensors/"+name+"/step_start",209)
        self.step_value = rospy.get_param("~sensors/"+name+"/step_value",21)
        self.step_count = rospy.get_param("~sensors/"+name+"/step_count",30)

        # sensor type: choices are A710YK (40-216"), A02YK (8-60"), A21YK (4-30")
        self.sensor = rospy.get_param("~sensors/"+name+"/sensor_type","A710YK")
        if self.sensor == "A710YK" or self.sensor == "ultralong":
            self.conversion = self.gpA710YK
        elif self.sensor == "A02YK" or self.sensor == "long":
            self.conversion = self.gpA02YK
        else:
            self.conversion = self.gpA21YK
        self.out_of_range = rospy.get_param("~sensors/"+name+"/out_of_range",0.0)
        
        # annoyingly loud, allow servo panning to be turned on/off
        self.enable = rospy.get_param("~sensors/"+name+"/enabled",False)
        rospy.Service('EnablePML',Empty,self.enable_callback)   
        rospy.Service('DisablePML',Empty,self.disable_callback)

        # publisher
        self.scanPub = rospy.Publisher('base_scan', LaserScan)

        rospy.loginfo("Started pml sensor '"+name+"' using servo: " + str(self.servo_id))

    def run(self):
        # set PML servo, sensor IDs, make sure it's disabled
        self.device.write(253, self.PML_SERVO, [self.servo_id, self.sensor_id])
        self.device.write(253, self.PML_MIN_L, [self.step_start%256, self.step_start>>8, self.step_value, self.step_count])
        self.device.write(253, self.PML_ENABLE, [0])
        # run
        r = rospy.Rate(self.rate * 8)   # 8 times oversampled...
        d = self.DN_SCAN
        while not rospy.is_shutdown():
            if self.enable:
                # check if new data
                if self.device.read(253,self.PML_DIR,1)[0] == d:                
                    # get ranges
                    v = self.device.read(253, self.PML_DIR, 3+self.step_count*2) 
                    offset =  (v[1]+(v[2]<<8))/1000.0
                    ranges = list()
                    for i in range(self.step_count):
                        if d == self.UP_SCAN:
                            k = v[i*2+3] + (v[i*2+4]<<8)
                        else:
                            k = v[(self.step_count*2+1)-i*2] + (v[(self.step_count*2+2)-i*2]<<8)
                        ranges.append( self.conversion(k) )
                    # now post laser scan
                    scan = LaserScan()
                    scan.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(offset)      
                    scan.header.frame_id = self.frame_id
                    if d == self.UP_SCAN:
                        scan.angle_min = (self.step_start-512) * 0.00511
                        scan.angle_max = (self.step_start+self.step_value*(self.step_count-1)-512) * 0.00511
                        scan.angle_increment = self.step_value * 0.00511
                    else:
                        scan.angle_min = (self.step_start+self.step_value*(self.step_count-1)-512) * 0.00511
                        scan.angle_max = (self.step_start-512) * 0.00511
                        scan.angle_increment = -self.step_value * 0.00511
                    scan.scan_time = offset #self.rate
                    scan.range_min = 0.5
                    scan.range_max = 6.0
                    scan.ranges = ranges    
                    self.scanPub.publish(scan)
                    if d == self.UP_SCAN: d = self.DN_SCAN
                    else: d = self.UP_SCAN
            r.sleep()
        # it's annoying, turn it off before we leave!
        self.disable_callback(None)

    def enable_callback(self, req):
        rospy.loginfo("PML Enabled")
        self.device.write(253, self.PML_ENABLE, [1])
        self.enable = True
        return EmptyResponse()
    
    def disable_callback(self, req):
        rospy.loginfo("PML Disabled")
        self.enable = False
        self.device.write(253, self.PML_ENABLE, [0])
        return EmptyResponse()

    # voltage to distance conversion functions
    def gpA710YK(self, value):
        if value > 100:
            return (497.0/(value-56))
        else:
            return self.out_of_range
    def gpA02YK(self, value):
        if value > 80:
            return (115.0/(value-12))
        else:
            return self.out_of_range 
    def gpA21YK(self, value):
        if value > 40:
            return (52.0/(value-12))
        else:
            return self.out_of_range 
