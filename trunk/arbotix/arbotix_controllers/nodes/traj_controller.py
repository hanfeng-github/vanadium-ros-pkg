#!/usr/bin/env python

"""
  traj_controller.py - controls joints using trajectory msgs
  Copyright (c) 2010-2010 Vanadium Labs LLC.  All right reserved.

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
import roslib; roslib.load_manifest('arbotix_controllers')
import rospy

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64

class TrajController:
    """ Controller to handle trajectory-based servo control. """

    def __init__(self):
        rospy.init_node("traj_joint_controller")
        self.joints = rospy.get_param("~joints")

        # subscriptions
        rospy.Subscriber('~command', JointTrajectory, self.cmdTrajCb)
        
        # publishers
        self.publishers = dict()
        for name in self.joints:
            self.publishers[name] = rospy.Publisher(name+"/command", Float64)
        rospy.loginfo("Started joint_controller controlling: " + str(self.joints))
                
        # TODO: do interpolation
        rospy.spin()

    def cmdTrajCb(self, msg):
        """ Store trajectories. """
        i = 0
        for joint in msg.joint_names:
            if joint in self.joints:      
                m = Float64(msg.points[0].positions[i])
                self.publishers[joint].publish(m)
            i += 1

if __name__=="__main__":
    tc = TrajController()
