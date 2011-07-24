#!/usr/bin/env python

"""
  traj_controller.py - position control of joints using trajectory msgs
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

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
import roslib; roslib.load_manifest('arbotix_controllers')
import rospy
import thread

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class Trajectory:
    def __init__(self, names, positions, velocities, accelerations, stime, etime):
        self.names = names
        self.positions = positions
        self.velocities = velocities
        self.accelerations = accelerations
        self.stime = stime
        self.etime = etime

class TrajControllerP:
    """ Position-Based Trajectory Controller to handle trajectory-based servo control. """

    def __init__(self):
        rospy.init_node("traj_controller")
        self.joints = rospy.get_param("~joints")
        self.rate = float(rospy.get_param("~rate", 50))

        self.mutex = thread.allocate_lock()

        # joint values
        self.valid = False
        self.dirty = False
        self.current = [0.0 for i in self.joints]
        self.last = [0.0 for i in self.joints]
        self.desired = [0.0 for i in self.joints]   
        self.velocity = [0.0 for i in self.joints]   # velocity to make next position setpoint
        self.trajectories = list()  
        self.endtime = rospy.Time.now()   

        # subscriptions
        rospy.Subscriber('~command', JointTrajectory, self.cmdTrajCb)
        rospy.Subscriber('joint_states', JointState, self.stateCb)
        self.publishers = dict(zip(self.joints, [rospy.Publisher(name+"/command", Float64) for name in self.joints]))
        
        rospy.loginfo("Started joint_controller controlling: " + str(self.joints))

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # grab mutex
            self.mutex.acquire()  

            # remove missed frames
            now = rospy.Time.now()
            while len(self.trajectories) > 1 and self.trajectories[0].etime < now:
                print "loop: deleting trajectory"
                del self.trajectories[0]

            # update desired/velocity
            if self.trajectories and now >= self.trajectories[0].stime: 
                # copy trajectory to desired/velocity arrays
                traj = self.trajectories[0]
                self.endtime = traj.etime
                try:
                    indexes = [traj.names.index(name) for name in self.joints]
                except ValueError as val:
                    rospy.logerr('Invalid trajectory. Aborting.')
                    return           
                self.desired = [ traj.positions[k] for k in indexes ]
                del self.trajectories[0]                 
                self.last = [i for i in self.current]       
                err = [ (d-c) for d,c in zip(self.desired,self.last) ]
                self.velocity = [ abs(x / (self.rate * (self.endtime - now).to_sec())) for x in err ]   
                #self.last = [i for i in self.current]
                self.dirty = True

            if self.dirty and self.valid:
                # interpolate and update outputs
                cumulative = 0.0
                err = [ (d-c) for d,c in zip(self.desired,self.last) ] #self.current) ]
                rospy.loginfo(err)
                for i in range(len(self.joints)):
                    #if err[i] > self.velocity[i] or err[i] < -self.velocity[i]:
                    if err[i] > 0.01 or err[i] < -0.01:
                        cumulative += abs(err[i])
                    #    time = (self.endtime - now).to_sec()
                    #    if time > 0:
                    #        velocity = err[i] / (self.rate * time)
                    #    else:
                    #        velocity = err[i]
                        cmd = err[i] 
                        top = self.velocity[i]
                        if cmd > top:
                            cmd = top
                        elif cmd < -top:
                            cmd = -top
                        self.last[i] += cmd
                        self.publishers[self.joints[i]].publish(Float64(self.last[i]))
                    else:
                        self.last[i] = self.desired[i]
                # are we done?
                rospy.loginfo(cumulative)
                if cumulative == 0.0:
                    self.dirty = False   
            # release mutex
            self.mutex.release()       
            rate.sleep()

    def restart(self):
        """ Restart the controller, by clearing out trajectory lists. """
        self.trajectories = list()                 
        self.dirty = False
        rospy.loginfo("Aborting trajectory, restart")

    def cmdTrajCb(self, msg):
        """ Store trajectories. """
        # grab mutex
        self.mutex.acquire()  
        # Stop?
        if len(msg.points) == 0:
            self.restart()
        else:       
            start = msg.header.stamp
            rospy.loginfo(start.to_sec())
            # remove trajectories after start of this trajectory
            while self.trajectories and self.trajectories[-1].etime > start:
                rospy.loginfo("Delete trajectory that ends at "+str(self.trajectories[-1].etime))
                del self.trajectories[-1]
            # fo' each point in trajectory
            for point in msg.points:
                # find start time of point
                t = msg.header.stamp + point.time_from_start
                self.trajectories.append(Trajectory(msg.joint_names, point.positions, point.velocities, point.accelerations, start, t))
                start = t
        # release mutex
        self.mutex.release()

    def stateCb(self, msg):
        """ The callback that converts JointState into servo position for interpolation. """
        # grab mutex
        self.mutex.acquire()  
        try:
            indexes = [msg.name.index(name) for name in self.joints]
        except ValueError as val:
            self.valid = False
            rospy.logerr('Invalid joint state message.')
            return
        self.current = [ msg.position[k] for k in indexes ]
        self.valid = True
        # release mutex
        self.mutex.release()

if __name__=="__main__":
    tc = TrajControllerP()

