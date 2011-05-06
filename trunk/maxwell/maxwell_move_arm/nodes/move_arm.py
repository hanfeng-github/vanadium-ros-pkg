#!/usr/bin/env python

"""
  move_arm.py - this is a simple move_arm alternative with no trajectory checking
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

import roslib; roslib.load_manifest('maxwell_move_arm')
import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from kinematics_msgs.msg import KinematicSolverInfo, PositionIKRequest
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionIKRequest
from maxwell_move_arm.srv import *

from math import *

class MoveArmServer:
    """ A class to move Maxwell's 5DOF arm to a designated location (x,y,z), 
        respecting wrist rotation (roll) and gripper rotation (pitch). As the 
        arm is 5DOF, we ignore the yaw input.

        Additional measures to improve IK abilities (increase search space):
         1) Iteratively adjust gripper pitch to try to find solutions.
         2) If gripper pitch is not enough, move base!

        To accomplish this, we might want to find an approximation of the current
        search space limitations for the arm. 
    """

    def __init__(self):
        rospy.init_node("simple_move_arm")        
        
        # connect to arm kinematics
        rospy.wait_for_service('arm_kinematics/get_ik')
        rospy.wait_for_service('arm_kinematics/get_ik_solver_info')
        self._get_ik_proxy = rospy.ServiceProxy('arm_kinematics/get_ik', GetPositionIK, persistent=True)
        self._get_ik_solver_info_proxy = rospy.ServiceProxy('arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)

        # setup tf for translating poses
        self._listener = tf.TransformListener()

        # a publisher for arm movement
        self._pub = rospy.Publisher('arm_controller/command', JointTrajectory)

        # listen to joint states
        self.servos = dict()
        rospy.Subscriber('joint_states', JointState, self.stateCb)

        # load service for moving arm
        rospy.Service('move_arm/move', MoveArm, self.moveCb)
        rospy.Service('move_arm/halt', HaltArm, self.haltCb)

        rospy.loginfo('Simple move_arm service started')
        rospy.spin()

    def stateCb(self, msg):
        for i in range(len(msg.name)):
            joint = msg.name[i]
            self.servos[joint] = msg.position[i]

    def moveCb(self, req):
        """ Given pose to move gripper to, do it. """ 
        arm_solver_info = self._get_ik_solver_info_proxy()     
  
        # goal of this service call    
        pose = self._listener.transformPose("torso_link", req.pose_stamped)
        
        # create IK request
        request = GetPositionIKRequest()
        request.timeout = rospy.Duration(5.0)

        request.ik_request.pose_stamped.header.frame_id = "torso_link";
        request.ik_request.ik_link_name = "gripper_link";
        request.ik_request.pose_stamped.pose.position.x = pose.pose.position.x
        request.ik_request.pose_stamped.pose.position.y = pose.pose.position.y
        request.ik_request.pose_stamped.pose.position.z = pose.pose.position.z

        # 5DOF, so yaw angle = atan2(Y,X-shoulder offset)
        e = euler_from_quaternion([pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w])
        q =  quaternion_from_euler(e[0], e[1], atan2(pose.pose.position.y, pose.pose.position.x - 0.061))
        
        request.ik_request.pose_stamped.pose.orientation.x = q[0]
        request.ik_request.pose_stamped.pose.orientation.y = q[1]
        request.ik_request.pose_stamped.pose.orientation.z = q[2]
        request.ik_request.pose_stamped.pose.orientation.w = q[3]

        request.ik_request.ik_seed_state.joint_state.name = arm_solver_info.kinematic_solver_info.joint_names
        request.ik_request.ik_seed_state.joint_state.position = [0]*len(request.ik_request.ik_seed_state.joint_state.name )

        # get IK
        response = self._get_ik_proxy(request)
        rospy.loginfo(response)

        # move the arm -- basically no interpolation right now
        # TODO: linear interpolation
        if response.error_code.val == response.error_code.SUCCESS:
            arm_solver_info = self._get_ik_solver_info_proxy()     
            msg = JointTrajectory()
            msg.joint_names = arm_solver_info.kinematic_solver_info.joint_names
            msg.points = list()
            point = JointTrajectoryPoint()
            point.positions = [ 0.0 for servo in msg.joint_names ]
            point.velocities = [ 0.0 for servo in msg.joint_names ]
            dists = [0.0 for servo in msg.joint_names]
            for joint in request.ik_request.ik_seed_state.joint_state.name:
                i = msg.joint_names.index(joint)
                point.positions[i] = response.solution.joint_state.position[response.solution.joint_state.name.index(joint)] 
                point.velocities[i] = 0.2
                # find total travel distance right now
                try:
                    dists[i] = abs(point.positions[i] - self.servos[joint])
                except:
                    pass
            max_dist = max(dists)
            for i in range(len(dists)): #request.ik_request.ik_seed_state.joint_state.name:
                if dists[i] > 0:
                    point.velocities[i] = dists[i]/max_dist * 0.2

            msg.points.append(point)
            msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
            self._pub.publish(msg)
            return MoveArmResponse(True)
        else:
            return MoveArmResponse(False)
   
    def haltCb(self, req):
        ''' Publish empty trajectory to stop arm. '''
        self._pub.publish(JointTrajectory())
        return HaltArmResponse(True)

if __name__ == '__main__':
    try:
        MoveArmServer()
    except rospy.ROSInterruptException:
        rospy.loginfo("And that's all folks...")

