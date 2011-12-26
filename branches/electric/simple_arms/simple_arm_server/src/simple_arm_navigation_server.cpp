/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Vanadium Labs LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Vanadium Labs LLC nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * author: Michael Ferguson
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <simple_arm_server/MoveArmAction.h>
#include <std_msgs/Float64.h>
#include <math.h>

class SimpleArmNavigationServer
{
  private:
    ros::NodeHandle nh;

    // ROS interface
    actionlib::SimpleActionServer<simple_arm_server::MoveArmAction> server;
    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> client;
    ros::Publisher gripper;

    simple_arm_server::MoveArmFeedback     feedback_;
    simple_arm_server::MoveArmResult       result_;
    simple_arm_server::MoveArmGoalConstPtr goal_;

    // parameters
    double timeout_;

  public:
    
    SimpleArmNavigationServer() : nh("~"), server(nh, "move_arm", false), client("move_arm", true)
    {
        ROS_INFO("Starting simple_arm_navigation_server.");
        // configuration
        nh.param<double>("timeout", timeout_, 5.0);

        // output for arm movement
        client.waitForServer();
        gripper = nh.advertise<std_msgs::Float64>("/gripper_controller/command", 1, false);
        ROS_INFO("Connected to move_arm server");

        // input for moving arm
        server.registerGoalCallback(boost::bind(&SimpleArmNavigationServer::goalCB, this));
        server.registerPreemptCallback(boost::bind(&SimpleArmNavigationServer::preemptCB, this));
        server.start();
        
        ROS_INFO("simple_arm_navigation_server node started");
    }

    void goalCB()
    {
        // accept the new goal
        goal_ = server.acceptNewGoal();
        
        for(size_t i = 0; i < goal_->motions.size(); i++){
            simple_arm_server::ArmAction action = goal_->motions[i];

            if( action.type == simple_arm_server::ArmAction::MOVE_ARM )
            {
                arm_navigation_msgs::MoveArmGoal goal;
                        
                goal.motion_plan_request.group_name = "arm";
                goal.motion_plan_request.num_planning_attempts = 1;
                goal.motion_plan_request.planner_id = std::string("");
                goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
                goal.motion_plan_request.allowed_planning_time = ros::Duration(timeout_);

                arm_navigation_msgs::SimplePoseConstraint desired_pose;
                desired_pose.absolute_position_tolerance.x = 0.005;
                desired_pose.absolute_position_tolerance.y = 0.005;
                desired_pose.absolute_position_tolerance.z = 0.005;
                desired_pose.absolute_roll_tolerance = 0.25;
                desired_pose.absolute_pitch_tolerance = 0.25;
                desired_pose.absolute_yaw_tolerance = 1.57;

                desired_pose.header.frame_id = goal_->header.frame_id; //pose.header.frame_id;
                desired_pose.link_name = "gripper_link";
                desired_pose.pose = action.goal;
                arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose, goal);

                // send request
                client.sendGoal(goal);
                bool finished_within_time = client.waitForResult(ros::Duration(300.0));
                if( !finished_within_time )
                {
                    client.cancelGoal();
                    ROS_INFO("Continue: goal timed out");
                }
                    //result_.success = false;
                    //server.setAborted(result_);
                    //return;
            }
            else if( action.type == simple_arm_server::ArmAction::MOVE_GRIPPER )
            {
                ROS_INFO("Move gripper to %f.", action.command);
                std_msgs::Float64 msg;
                msg.data = action.command;
                gripper.publish( msg );
                ros::Duration(action.move_time).sleep();
            }
        } // for each action

        result_.success = true;
        server.setSucceeded(result_);
    }

    void preemptCB()
    {
        ROS_INFO("simple_move_arm preempted");
        server.setPreempted();
    }

};

int main(int argc, char **argv)
{
    ros::init (argc, argv, "simple_arm_navigation_server");
    
    SimpleArmNavigationServer server;
    ros::spin();
    
    return 0;
}

