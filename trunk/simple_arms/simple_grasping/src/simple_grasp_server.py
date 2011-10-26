#!/usr/bin/env python

"""
  simple_grasp_server.py - this is a simple way to move an arm
  Copyright (c) 2011 Michael Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holders nor the names of its 
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

import roslib; roslib.load_manifest("simple_grasping")
import rospy, tf

from simple_grasping.msg import *   # Pick, Place actions
from control_msgs.msg import *      # FollowJointTrajectory

from arm_navigation_msgs.msg import AttachedCollisionObject, CollisionObject

class SimpleGraspServer():
    """
        This server offers 2 actions for pick and place.
    """    

    def __init__(self):
        rospy.init_node("simple_grasp_server")
        
        # parameters
        self.arm = rospy.get_param("~arm_name", "arm")
        self.gripper = rospy.get_param("~gripper_name", "gripper")
        self.touch_links = rospy.get_param("~touch_links", [])

        # setup tf for translating poses
        self.listener = tf.TransformListener()

        # outputs
        self.client = actionlib.SimpleActionClient(self.arm+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()
        self.gripper = rospy.Publisher(self.gripper+'_controller/command', Float64)

        # inputs
        self.pick_server = actionlib.SimpleActionServer("~pick", PickAction, execute_cb=self.PickCb, auto_start=False)
        self.pick_server.start()
        self.place_server = actionlib.SimpleActionServer("~place", PlaceAction, execute_cb=self.PlaceCb, auto_start=False)
        self.place_server.start()

        rospy.loginfo('simple_grasp_server node started')
        rospy.spin()

    def PickCb(self, goal):
        if goal.arm_name != self.arm and self.arm != "":
            rospy.logerr("Goal requests an arm not under control of this node.")
        
        object_id = goal.object_id
        grasp = goal.grasps[0]  # TODO, fallback?

        #  1) pre-grasp call to arm_navigation

        #  2) interpolated IK towards grasp
        #disable collisions between object_id and gripper_links

        #  3) gripper closure

        # attach object to gripper
        obj = AttachedCollisionObject()
        obj.link_name = grasp.gripper_frame_id
        obj.object.
        obj.touch_links = self.touch_links

        #  4) retract gripper
        #disable collisions between object_id and collision_map


    def PlaceCb(self, goal):
        # place action:
        #  inputs:  PlaceAction: id of object
        #                        array of potential goal poses
        #                        gripper opening
        #
        #  outputs: 1) pre-grasp call to arm_navigation
        #           2) interpolated IK towards grasp
        #           3) gripper closure
        #           4) retract gripper

        pass


if __name__=="__main__":
    try:
        SimpleGraspServer()
    except rospy.ROSInterruptException:
        rospy.loginfo("And that's all folks...")

