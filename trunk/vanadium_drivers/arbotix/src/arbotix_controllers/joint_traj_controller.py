#!/usr/bin/env python

"""
  joint_traj_controller.py - controls joints using trajectory msgs
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
import thread

from threading import Thread
from trajectory_msgs.msg import JointTrajectory

class joint_traj_controller(Thread):
    """ Controller to handle basic servo control. """

    def __init__(self, device, name, joints=None):
        Thread.__init__ (self)
        
        # handle for robocontroller
        self.device = device
        self.name = name

        # joint trajectory storage
        self.time_traj  = dict()    
        self.mutex = thread.allocate_lock()

        # parameters
        self.joints = rospy.get_param("~controllers/"+name+"/joints")
        self.rate = rospy.get_param("~controllers/"+name+"/rate",15.0)

        # subscriptions
        rospy.Subscriber(name+'/command', JointTrajectory, self.cmdTrajCb)
        rospy.loginfo("Started joint_controller '"+name+"' controlling: " + str(self.joints))

    def restart(self):
        self.time_traj  = dict()    

    def run(self):
        """ Do joint interpolation. """
        rospy.spin()
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown):
            self.mutex.acquire()  

            self.mutex.release()
            r.sleep()
          
#            for joint in msg.joint_names:
#            if joint in self.joints:            
#                if self.mode == "servo":
#                    self.device.servos[joint].setAngle( msg.points[msg.joint_names.index(joint)].positions[0] )
#                else:
#                    pass


    def cmdTrajCb(self, msg):
        """ The callback that stores JointTrajectory updates. """
        # grab mutex
        self.mutex.acquire()  

        # Stop?
        if len(msg.points) == 0:
            self.restart()
        else:

            # find start time of this trajectory set
            # TODO

            # grabbed mutex, now process data, one trajectory at a time.
            #for i in range(len(msg.points):
            
            # total crap, first pass to see how this works with joystick teleop
            for i in range(len(msg.joint_names)):
                name = msg.joint_names[i]
                position = msg.points[0].positions[i]
                self.device.servos[name].setAngle( position )
        self.mutex.release()

