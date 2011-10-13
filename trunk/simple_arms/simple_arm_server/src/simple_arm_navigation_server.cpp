/* Michael E Ferguson */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
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
    tf::TransformListener tf_listener;

    // ROS interface
    actionlib::SimpleActionServer<simple_arm_server::MoveArmAction> server;
    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> client;
    ros::Publisher gripper;

    simple_arm_server::MoveArmFeedback     feedback_;
    simple_arm_server::MoveArmResult       result_;
    simple_arm_server::MoveArmGoalConstPtr goal_;

    // parameters
    int arm_dof_;
    std::string root_name_;
    std::string tip_name_;
    double timeout_;

  public:
    
    SimpleArmNavigationServer() : nh("~"), server(nh, "move_arm", false), client("move_arm", true)
    {
        ROS_INFO("Starting simple_arm_navigation_server.");
        // configuration
        nh.param<int>("arm_dof", arm_dof_, 5);
        nh.param<std::string>("root_name", root_name_, "arm_link");
        nh.param<std::string>("tip_name", tip_name_, "gripper_link");
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
                // construct the representation of a pose goal (define the position of the end effector)
                arm_navigation_msgs::MoveArmGoal goal;
    
                goal.motion_plan_request.group_name = "arm";
                goal.motion_plan_request.num_planning_attempts = 1;
                goal.motion_plan_request.planner_id = std::string("");
                goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
                goal.motion_plan_request.allowed_planning_time = ros::Duration(timeout_);
    
                arm_navigation_msgs::SimplePoseConstraint desired_pose;
                desired_pose.header.frame_id = root_name_; // TODO: goal_.header.frame_id?
                desired_pose.link_name = tip_name_;
                desired_pose.pose.position.x = action.goal.position.x;
                desired_pose.pose.position.y = action.goal.position.y;
                desired_pose.pose.position.z = action.goal.position.z;

                desired_pose.absolute_position_tolerance.x = 0.05;
                desired_pose.absolute_position_tolerance.y = 0.05;
                desired_pose.absolute_position_tolerance.z = 0.05;
        
                double roll, pitch, yaw;
                btQuaternion q(action.goal.orientation.x, action.goal.orientation.y, action.goal.orientation.z, action.goal.orientation.w);
                btMatrix3x3(q).getRPY(roll, pitch, yaw);
                if( arm_dof_ < 6 ){
                    // 5DOF, so yaw angle = atan2(Y,X-shoulder offset)
                    yaw = atan2(action.goal.position.y, action.goal.position.x);
                    if( arm_dof_ < 5 ){
                        // 4 DOF, so yaw as above AND no roll
                        roll = 0.0;
                    }
                }
                q.setRPY(roll, pitch, yaw);

                desired_pose.pose.orientation.x = (double) q.getX();
                desired_pose.pose.orientation.y = (double) q.getY();
                desired_pose.pose.orientation.z = (double) q.getZ();
                desired_pose.pose.orientation.w = (double) q.getW();
                ROS_INFO("%f, %f, %f", roll, pitch, yaw);

                desired_pose.absolute_roll_tolerance = 0.1;
                desired_pose.absolute_pitch_tolerance = 0.1;
                desired_pose.absolute_yaw_tolerance = 0.1;
    
                arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose, goal);

                // send request
                client.sendGoal(goal);
                bool finished_within_time = client.waitForResult(ros::Duration(1.5*timeout_));
                if( !finished_within_time )
                {
                    client.cancelGoal();
                    ROS_INFO("Timed out");
                }
                else
                {
                    ROS_INFO("Goal returned");
                    actionlib::SimpleClientGoalState state = client.getState();
	                if( state != actionlib::SimpleClientGoalState::SUCCEEDED )
                    {
                        result_.success = false;
                        server.setAborted(result_);
                        return;                    
                    }
                }
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
/*


        if (!turnOdom(goal_->turn_distance))
        { 
          as_.setAborted(result_);
          return;
        }
        
        if (driveForwardOdom(goal_->forward_distance))
          as_.setSucceeded(result_);
        else
          as_.setAborted(result_);

    
    // send the request to the action server
    if (nh.ok())
    {
	bool finished_within_time = false;
	move_arm.sendGoal(goal);
	finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
	if (!finished_within_time)
	{
	    move_arm.cancelGoal();
	    ROS_INFO("Timed out achieving goal");
	}
	else
	{
	    actionlib::SimpleClientGoalState state = move_arm.getState();
	    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
	    if(success)
		ROS_INFO("Action finished: %s",state.toString().c_str());
	    else
		ROS_INFO("Action failed: %s",state.toString().c_str());
	}
    }
*/

