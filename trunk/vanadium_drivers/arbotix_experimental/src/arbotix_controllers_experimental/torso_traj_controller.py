#!/usr/bin/env python

"""
  torso_traj_controller.py - controls a torso lift joint, consisting of linear actuator,
    motor controller, and analog feedback. 
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

import rospy, thread
from threading import Thread

from trajectory_msgs.msg import JointTrajectory

class torso_traj_controller(Thread):
    """ Controller to handle basic linear actuator control. """

    def __init__(self, device, name, joints=None):
        Thread.__init__ (self)

        # handle for robocontroller
        self.device = device
        self.name = name

        # parameters
        self.joint = rospy.get_param("~controllers/"+name+"/joint","torso_lift_joint")      
        self.pwm = rospy.get_param("~controllers/"+name+"/pwm",2)           # pwm channel
        self.a = rospy.get_param("~controllers/"+name+"/a",3)               # a channel
        self.b = rospy.get_param("~controllers/"+name+"/a",5)               # b channel
        self.an = rospy.get_param("~controllers/"+name+"/a",0)              # analog feedback channel
        self.base = rospy.get_param("~controllers/"+name+"/base",7)         # analog reading that corresponds to 0
        self.step = rospy.get_param("~controllers/"+name+"/step",782)       # analog increase per meter step         

        # internal variables
        self.goal_position = -1.0
        self.last_position = -1.0
        self.mutex = thread.allocate_lock()

        # subscriptions
        rospy.Subscriber(name+'/command', JointTrajectory, self.cmdTrajCb)
        rospy.loginfo("Started torso_traj_controller '"+name+"' controlling: " + str(self.joint))

    def run(self):
        """ Move torso as required. """
        r = rospy.Rate(5.0)
        # simple linear interpolation
        while not rospy.is_shutdown():      
            # update position
            v = self.device.getAnalog(self.an)
            self.last_position = (v-self.base)/float(self.step)
            self.device.servos[self.joint].angle = self.last_position
            # output updates to bot
            if self.goal_position >= 0.0:        
                self.mutex.acquire()  
                goal = (self.goal_position * self.step) + self.base      
                if v + 2 < goal:
                    # go up
                    self.device.setDigital(self.a, 0xff, 0xff)
                    self.device.setDigital(self.b, 0x00, 0xff)
                    self.device.setDigital(self.pwm, 0xff, 0xff)
                elif v - 2 > goal:
                    # go down
                    self.device.setDigital(self.a, 0x00, 0xff)
                    self.device.setDigital(self.b, 0xff, 0xff)
                    self.device.setDigital(self.pwm, 0xff, 0xff)
                else:
                    self.device.setDigital(self.pwm,0)
                    self.goal_position = -1.0
                self.mutex.release()
            r.sleep()

    def cmdTrajCb(self, msg):
        """ The callback that converts JointTrajectory into torso movement. This does no 
            smoothing or interpolation, as we aren't doing PWM control of linear actuator."""
        self.mutex.acquire()  
        if msg.joint_names[0] == self.joint:
            self.goal_position = msg.points[0].positions[-1]
#            print "Setting torso position to " + str(self.goal_position) + " " + str(self.last_position)
        self.mutex.release()

