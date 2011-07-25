#!/usr/bin/env python

"""
  pml.py - the Planer Meta-Laser (formerly Poor Man's Scanning Laser)
  Copyright (c) 2008-2011 Vanadium Labs LLC.  All right reserved.

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

from sensor_msgs.msg import LaserScan
from arbotix_msgs.msg import ScanParameters
from std_srvs.srv import *

from arbotix_python.sensors import *

class pml:
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

    def __init__(self, device):
        # handle for robocontroller
        self.device = device

        # parameters
        self.throttle = int(device.rate/rospy.get_param("~pml/rate",5))
        self.frame_id = rospy.get_param("~pml/frame","base_laser") 
        self.servo_id = rospy.get_param("~pml/servo_id",200) 
        self.sensor_id = rospy.get_param("~pml/sensor_id",0)

        self.step_start = rospy.get_param("~pml/step_start",209)
        self.step_value = rospy.get_param("~pml/step_value",21)
        self.step_count = rospy.get_param("~pml/step_count",30)

        # sensor type: choices are A710YK (40-216"), A02YK (8-60"), A21YK (4-30")
        self.sensor_t = rospy.get_param("~pml/sensor_type","A710YK")
        if self.sensor_t == "A710YK" or self.sensor_t == "ultralong":
            self.sensor = gpA710YK()
        elif self.sensor_t == "A02YK" or self.sensor_t == "long":
            self.sensor = gpA02YK()
        else:
            self.sensor = gp2d12()
        
        # annoyingly loud, allow servo panning to be turned on/off
        self.enable = rospy.get_param("~pml/enabled",False)
        rospy.Service('EnablePML',Empty,self.enableCb)   
        rospy.Service('DisablePML',Empty,self.disableCb)

        # publishers and subscribers
        self.scanPub = rospy.Publisher('base_scan', LaserScan)
        rospy.Subscriber('scan_parameters',ScanParameters,self.paramCb)

        rospy.loginfo("Started pml sensor using servo: " + str(self.servo_id))

    def update(self):
        if self.enable:
            # check if new data
            try:
                if self.device.read(253,self.PML_DIR,1)[0] == self.d:                
                    # get ranges
                    v = self.device.read(253, self.PML_DIR, 3+self.step_count*2) 
                    offset =  (v[1]+(v[2]<<8))/1000.0
                    ranges = list()
                    for i in range(self.step_count):
                        if self.d == self.UP_SCAN:
                            k = v[i*2+3] + (v[i*2+4]<<8)
                        else:
                            k = v[(self.step_count*2+1)-i*2] + (v[(self.step_count*2+2)-i*2]<<8)
                        ranges.append( self.sensor.convert(k) )
                    # now post laser scan
                    scan = LaserScan()
                    scan.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(offset)      
                    scan.header.frame_id = self.frame_id
                    if self.d == self.UP_SCAN:
                        scan.angle_min = (self.step_start-512) * 0.00511
                        scan.angle_max = (self.step_start+self.step_value*(self.step_count-1)-512) * 0.00511
                        scan.angle_increment = self.step_value * 0.00511
                    else:
                        scan.angle_min = (self.step_start+self.step_value*(self.step_count-1)-512) * 0.00511
                        scan.angle_max = (self.step_start-512) * 0.00511
                        scan.angle_increment = -self.step_value * 0.00511
                    scan.scan_time = offset
                    scan.time_increment = offset/(self.step_count-1)
                    scan.range_min = self.sensor.min_range
                    scan.range_max = self.sensor.max_range
                    scan.ranges = ranges    
                    self.scanPub.publish(scan)
                    if self.d == self.UP_SCAN: self.d = self.DN_SCAN
                    else: self.d = self.UP_SCAN
            except:
                rospy.logerr("Dropped a PML packet")

    def startup(self):
        # set PML servo, sensor IDs, make sure it's disabled
        self.device.write(253, self.PML_SERVO, [self.servo_id, self.sensor_id])
        self.device.write(253, self.PML_MIN_L, [self.step_start%256, self.step_start>>8, self.step_value, self.step_count])
        self.device.write(253, self.PML_ENABLE, [0])
        self.d = self.DN_SCAN

    def shutdown(self):
        # it's annoying, turn it off before we leave!
        self.disableCb(None)

    def enableCb(self, req):
        rospy.loginfo("PML Enabled")
        self.device.write(253, self.PML_ENABLE, [1])
        self.enable = True
        return EmptyResponse()
    
    def disableCb(self, req):
        rospy.loginfo("PML Disabled")
        self.enable = False
        self.device.write(253, self.PML_ENABLE, [0])
        return EmptyResponse()

    def paramCb(self, req):
        """ Process updates to scan parameters. """
        self.step_start = 512 + int(req.angle_min*195.56)   # convert radians to ticks
        self.step_count = req.readings
        self.step_value = int(req.angle_increment*195.56)
        rospy.loginfo("Setting scan parameters to: " + str(self.step_start) + "," + str(self.step_value) + "," + str(self.step_count) )
        self.device.write(253, self.PML_MIN_L, [self.step_start%256, self.step_start>>8, self.step_value, self.step_count])
