#!/usr/bin/env python

"""
  diagnostics.py - diagnostic output code
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
from diagnostic_msgs.msg import *
from arbotix_python.ax12 import P_PRESENT_VOLTAGE

class DiagnosticsPublisher:

    def __init__(self, device):
        self.device = device
        self.t_delta = rospy.Duration(1.0/rospy.get_param("~diagnostic_rate", 1.0))
        self.t_next = rospy.Time.now() + self.t_delta
        self.pub = rospy.Publisher('diagnostics', DiagnosticArray)

    def update(self):
        """ Publish diagnostics. """    
        if rospy.Time.now() > self.t_next:    
            # update status of servos
            if self.device.use_sync_read:
                # arbotix/servostik/wifi board sync_read
                synclist = list()
                for servo in self.device.dynamixels.values():
                    if servo.readable:
                        synclist.append(servo.id)
                    else:
                        servo.update(-1)
                if len(synclist) > 0:
                    val = self.device.syncRead(synclist, P_PRESENT_VOLTAGE, 2)
                    if val: 
                        for servo in self.device.dynamixels.values():
                            try:
                                i = synclist.index(servo.id)*2
                                servo.voltage = val[i]/10.0
                                servo.temperature = val[i+1]
                            except:
                                # not a readable servo
                                continue 
            else:
                # direct connection, or other hardware with no sync_read capability
                for servo in self.device.dynamixels.values():
                    if servo.readable:
                        try:
                            val = self.device.read(servo.id, P_PRESENT_VOLTAGE, 2)
                            servo.voltage = val[0]
                            servo.temperature = val[1]
                        except:
                            pass
            # publish diagnostics data
            msg = DiagnosticArray()
            msg.header.stamp = rospy.Time.now()
            for servo in self.device.dynamixels.values():
                stat = DiagnosticStatus()
                stat.name = "Joint " + servo.name
                stat.level = DiagnosticStatus.OK
                stat.message = "OK"
                stat.values.append(KeyValue("Position", str(servo.angle)))
                stat.values.append(KeyValue("Temperature", str(servo.temperature)))
                if servo.temperature > 60:
                    stat.level = DiagnosticStatus.ERROR
                    stat.message = "OVERHEATED"
                elif servo.temperature > 50:
                    stat.level = DiagnosticStatus.WARN
                    stat.message = "VERY HOT"
                stat.values.append(KeyValue("Voltage", str(servo.voltage)))
                if servo.relaxed:
                    stat.values.append(KeyValue("Torque", "OFF"))
                else:
                    stat.values.append(KeyValue("Torque", "ON"))
                msg.status.append(stat)
    #        if self.device.base:
    #            stat = DiagnosticStatus()
    #            stat.name = "Encoders"
    #            stat.level = DiagnosticStatus.OK
    #            stat.message = "OK"
    #            stat.values.append(KeyValue("Left", str(self.base.enc_left)))
    #            stat.values.append(KeyValue("Right", str(self.base.enc_right)))
    #            msg.status.append(stat)
            self.pub.publish(msg)
            self.t_next = rospy.Time.now() + self.t_delta
        

