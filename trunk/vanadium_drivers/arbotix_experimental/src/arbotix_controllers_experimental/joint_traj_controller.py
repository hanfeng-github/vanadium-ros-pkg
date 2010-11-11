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

import rospy, thread
from threading import Thread

from trajectory_msgs.msg import JointTrajectory

class joint_traj_controller(Thread):
    """ Controller to handle trajectory-based servo control. """

    def __init__(self, device, name):
        Thread.__init__ (self)
        
        # handle for robocontroller
        self.device = device
        self.name = name

        # joint trajectory storage, [time] = {servo: pos, servo: pos, ...}
        self.trajectories = list()
        self.time_traj  = dict()    
        self.mutex = thread.allocate_lock()

        # parameters
        self.joints = rospy.get_param("~controllers/"+name+"/joints")
        self.rate = rospy.get_param("~controllers/"+name+"/rate",15.0)

        # subscriptions and services
        rospy.Subscriber(name+'/command', JointTrajectory, self.cmdTrajCb)
        rospy.loginfo("Started joint_controller '"+name+"' controlling: " + str(self.joints))

    def restart(self):
        """ Restart the controller, by clearing out trajectory lists. """
        self.trajectories = list()    
        self.time_traj  = dict()    
        print self.name+" aborting trajectory, restart"

    def run(self):
        """ Do joint interpolation. """
        r = rospy.Rate(self.rate)
        # simple linear interpolation
        while not rospy.is_shutdown():        
            self.mutex.acquire()  
            # remove missed frames
            now = rospy.Time.now()
            while self.trajectories and self.trajectories[0] < now:
                print self.name+": delete trajectory at "+str(self.trajectories[0])
                del self.time_traj[self.trajectories[0]]
                del self.trajectories[0]
            if self.trajectories:    
                # should this frame be output now?
                if self.trajectories[0] > now + rospy.Duration(1/self.rate):
                    print "Skipping this iteration, wait for next frame"
                else:
                    time = self.trajectories[0]
                    traj = self.time_traj[time]
                    # update servos
                    for servo in traj.keys():
                        self.device.servos[servo].setAngle( traj[servo] )
                    # clean up
                    del self.trajectories[0]
                    del self.time_traj[time]
            self.mutex.release()
            r.sleep()

    def cmdTrajCb(self, msg):
        """ The callback that stores JointTrajectory updates. """
        # grab mutex
        self.mutex.acquire()  
        # Stop?
        if len(msg.points) == 0:
            self.restart()
        else:
            # find start time of this trajectory set
            start = msg.header.stamp
            # remove trajectories after start
            while self.trajectories and self.trajectories[-1] > start + msg.points[0].time_from_start:
                print self.name+": delete trajectory at "+str(self.trajectories[-1])
                del self.time_traj[self.trajectories[-1]]
                del self.trajectories[-1]
            # insert trajectories
            for point in range(len(msg.points)):
                points = msg.points[point]
                time = start + points.time_from_start
                #print self.name+": append trajectory at "+str(time)
                if time > rospy.Time.now(): # + rospy.Duration(1/self.rate):
                    self.trajectories.append(time)
                    traj = dict()
                    for servo in range(len(msg.joint_names)):
                        traj[msg.joint_names[servo]] = points.positions[servo]
                    self.time_traj[time] = traj
            self.trajectories = sorted(self.trajectories)
        # release mutex
        self.mutex.release()

