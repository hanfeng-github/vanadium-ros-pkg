#!/usr/bin/env python

"""
  follow_controller.py - controller for a kinematic chain
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
from control_msgs.msg import FollowJointTrajectoryAction
from diagnostic_msgs.msg import *

from ax12 import *

class FollowController:
    """ A controller for joint chains, exposing a FollowJointTrajectory action. """

    def __init__(self, device, name):
        self.name = name
        self.device = device   
        self.active = False  
        self.interpolating = 0

        # parameters
        self.rate = rospy.get_param('~controllers/'+name+'/rate',50.0)
        self.joints = rospy.get_param('~controllers/'+name+'/joints')
        self.index = rospy.get_param('~controllers/'+name+'/index', 0)
        self.onboard = rospy.get_param('~controllers/'+name+'/onboard', True)
        for joint in self.joints:
            self.device.servos[joint].controller = self
        self.ids = [self.device.servos[joint].id for joint in self.joints]
        
        self.joint_names = []
        self.joint_positions = []
        self.joint_velocities = []

        # action server
        name = rospy.get_param('~controllers/'+name+'/action_name','follow_joint_trajectory')
        self.server = actionlib.SimpleActionServer(name, FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)

        rospy.loginfo("Started FollowController controlling: " + str(self.joints))

    def actionCb(self, goal):
        self.active = True
        traj = goal.trajectory

        if set(self.joints) != set(traj.joint_names):
            msg = "Trajectory joint names does not match action controlled joints."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            self.active = False
            return

        if not traj.points:
            msg = "Trajectory empy."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            self.active = False
            return

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            msg = "Trajectory invalid."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            self.active = False
            return    

        # carry out trajectory
        time = rospy.Time.now()
        start = traj.header.stamp
        r = rospy.Rate(1.0/self.rate)
        for point in traj.points:
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)
            if self.onboard:
                while self.interpolating > 0: 
                    pass
                positions = [ self.device.servos[self.joints[k]].setControl(point.positions[indexes[k]]) for k in range(len(indexes)) ]
                self.write(positions, point.time_from_start.to_sec())
            else:
                last = [ self.servos[joint] for joint in self.joints ]
                desired = [ point.positions[k] for k in indexes ]
                endtime = start + point.time_from_start
                while rospy.Time.now() + rospy.Duration(0.01) < endtime:
                    err = [ (d-c) for d,c in zip(desired,last) ]
                    velocity = [ abs(x / (self.rate * (endtime - rospy.Time.now()).to_sec())) for x in err ]   
                    rospy.loginfo(err)
                    for i in range(len(self.joints)):
                        if err[i] > 0.01 or err[i] < -0.01:
                            cmd = err[i] 
                            top = velocity[i]
                            if cmd > top:
                                cmd = top
                            elif cmd < -top:
                                cmd = -top
                            last[i] += cmd
                            self.servos[self.joints[i]].desired = last[i]
                            self.servos[self.joints[i]].dirty = True
                        else:
                            velocity[i] = 0
                    r.sleep()

        while self.onboard and self.interpolating != 0:
            pass

        self.active = False
        self.server.set_succeeded()
                
    def update(self):
        if self.active:
            self.status()

    def startup(self):
        self.setup() 
        self.server.start()
    
    def shutdown(self):
        pass

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if self.server.is_active():
            msg.values.append(KeyValue("State", "Active"))
        else:
            msg.values.append(KeyValue("State", "Not Active"))
        return msg

    ###
    ### Controller Specification: 
    ###
    ###  setup: list of controller ids
    ###
    ###  write: position of each servo (2 bytes), number of frames
    ###
    ###  status: interpolation status
    ### 
    
    def setup(self):
        params = [self.index] + self.ids
        success = self.device.execute(253, AX_CONTROL_SETUP, params)

    def write(self, positions, time):
        """ Write a movement to occur in time (s). """
        self.interpolating = 1
        params = [self.index]
        for p in positions:
            params.append( int(p)%256 )
            params.append( (int(p)>>8)%256 )
        params.append(int(time*30))
        success = self.device.execute(253, AX_CONTROL_WRITE, params)

    def status(self):
        try:
            self.interpolating = self.device.execute(253, AX_CONTROL_STAT, [self.index])[0]
        except:
            pass

