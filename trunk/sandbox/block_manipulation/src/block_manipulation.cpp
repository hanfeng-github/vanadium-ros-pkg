/* 
 * Copyright (c) 2011, Vanadium Labs LLC
 * All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Vanadium Labs LLC nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: Michael Ferguson
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <simple_arm_server/MoveArm.h>
#include <simple_arm_server/ArmAction.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <interactive_markers/interactive_marker_server.h>

#include <vector>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::ServiceClient client;
ros::Publisher pub_;
tf::TransformListener * tf_listener_;
int markers_;
int moving_;
int skip_;
float x_, y_;
std::string last_block_;

/*
 * Block Storage
 */
class Block
{
  public:
    int id;
    bool active;
    double x;
    double y;
    
    Block(const Block& b) : id(b.id), active(b.active), x(b.x), y(b.y) {}
    Block(int _id, double _x, double _y) : id(_id) , active(true), x(_x), y(_y) {}

    std::string getName()
    {
      std::stringstream conv;
      conv << id;
      return std::string("block") + conv.str(); 
    }
};
std::vector<Block> marker_names_;


/* 
 * Move the real block!
 */
void moveBlock( const InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM("Staging " << feedback->marker_name);     
      x_ = feedback->pose.position.x;
      y_ = feedback->pose.position.y;
      last_block_ = feedback->marker_name;
      break;
 
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      moving_ = true;
      ROS_INFO_STREAM("Now moving " << feedback->marker_name); 

      simple_arm_server::MoveArm srv;
      simple_arm_server::ArmAction * action = new simple_arm_server::ArmAction();
      
      /* arm straight up */
      btQuaternion temp;
      temp.setRPY(0,1.57,0);
      action->goal.orientation.x = temp.getX();
      action->goal.orientation.y = temp.getY();
      action->goal.orientation.z = temp.getZ();
      action->goal.orientation.w = temp.getW();

      /* hover over */
      action->goal.position.x = x_;
      action->goal.position.y = y_;
      action->goal.position.z = 0.08;
      srv.request.goals.push_back(*action);
      action->move_time.sec = 1.5;

      /* go down */
      action->goal.position.z = 0.03;
      srv.request.goals.push_back(*action);
      action->move_time.sec = 1.5;

      /* close gripper */
      simple_arm_server::ArmAction * grip = new simple_arm_server::ArmAction();
      grip->type = simple_arm_server::ArmAction::MOVE_GRIPPER;
      grip->command = 0.024;
      grip->move_time.sec = 1.0;
      srv.request.goals.push_back(*grip);

      /* go up */
      action->goal.position.z = 0.08;
      srv.request.goals.push_back(*action);
      action->move_time.sec = 0.25;

      /* hover over */
      action->goal.position.x = feedback->pose.position.x;
      action->goal.position.y = feedback->pose.position.y;
      action->goal.position.z = 0.08;
      srv.request.goals.push_back(*action);
      action->move_time.sec = 1.5;

      /* go down */
      action->goal.position.z = 0.03;
      srv.request.goals.push_back(*action);
      action->move_time.sec = 1.5;

      /* open gripper */
      grip->command = 0.040;
      srv.request.goals.push_back(*grip);

      /* go up */
      action->goal.position.z = 0.08;
      srv.request.goals.push_back(*action);
      action->move_time.sec = 0.25;

      srv.request.header.frame_id="base_link";
      client.call(srv);
      /* update location */ 
      for( std::vector<Block>::iterator it=marker_names_.begin(); it < marker_names_.end(); it++)
      {
        if( it->getName() == feedback->marker_name )
        {
          it->x = feedback->pose.position.x;
          it->y = feedback->pose.position.y;
          break;
        }
      }

      moving_ = false;
      break;
  }
  
  server->applyChanges(); 
}

/* 
 * Make a box
 */
Marker makeBox( InteractiveMarker &msg, float r, float g, float b )
{
  Marker m;

  m.type = Marker::CUBE;
  m.scale.x = msg.scale;
  m.scale.y = msg.scale;
  m.scale.z = msg.scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0;

  return m;
}
 
/* 
 * Add a new block
 */
void addBlock( float x, float y, float z, float rz, float r, float g, float b, int n)
{
  InteractiveMarker marker;
  marker.header.frame_id = "/base_link";
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.scale = 0.03;
  
  Block block( n, x, y );
  marker_names_.push_back( block );
  marker.name = block.getName(); 
  marker.description = "Another block";

  InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  marker.controls.push_back( control );
  
  control.markers.push_back( makeBox(marker, r, g, b) );
  control.always_visible = true;
  marker.controls.push_back( control );
  

  server->insert( marker );
  server->setCallback( marker.name, &moveBlock );
}

/* 
 * Process an incoming cloud
 */
void cloudCb ( const sensor_msgs::PointCloud2ConstPtr& msg )
{
  if( (skip_++)%15 != 0 ) return;

  // convert to PCL
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg (*msg, cloud);
  
  // transform to base_link
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!pcl_ros::transformPointCloud (std::string("base_link"), cloud, *cloud_transformed, *tf_listener_))
  {
    ROS_ERROR ("Error converting to base_link");
    return;
  }

  // drop things on ground
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_transformed); 
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.015, 0.1);
  pass.filter(*cloud_filtered);
  if( cloud_filtered->points.size() == 0 ){
    ROS_ERROR("0 points left");
    return;
  }else
    ROS_INFO("Filtered, %d points left", (int) cloud_filtered->points.size());
  pub_.publish(*cloud_filtered);

  // cluster
  pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud_filtered);
  
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(20);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(clusters);

  // need to delete old blocks
  for( std::vector<Block>::iterator it=marker_names_.begin(); it < marker_names_.end(); it++)
  {
    it->active = false;
  }

  // for each cluster, see if it is a block
  for (size_t c = 0; c < clusters.size (); ++c)
  {  
    // find cluster centroid/color
    float x = 0; float y = 0; float z = 0; int r = 0; int g = 0; int b = 0;
    for (size_t i = 0; i < clusters[c].indices.size(); i++)
    {
        int j = clusters[c].indices[i];
        x += cloud_filtered->points[j].x;
        y += cloud_filtered->points[j].y;
        z += cloud_filtered->points[j].z;
    }
    x = x/clusters[c].indices.size();
    y = y/clusters[c].indices.size();
    z = z/clusters[c].indices.size();

    bool new_ = true;
    // see if we have it detected
    for( std::vector<Block>::iterator it=marker_names_.begin(); it < marker_names_.end(); it++)
    {
      if( (fabs(it->x - x) < 0.0254) &&
          (fabs(it->y - y) < 0.0254) )
      {
        new_ = false;
        it->active = true;
        break;
      }
    }

    if (new_){
      // else, add new block
      addBlock( x, y, 0.0127, 0.0, (float) r/255.0, (float) g/255.0, (float) b/255.0, markers_++ );
    }
  }

  // need to delete old blocks
  for( std::vector<Block>::iterator it=marker_names_.begin(); it < marker_names_.end(); )
  {
    if(it->active or it->getName() == last_block_){
      it++;
    }else{
      server->erase( it->getName() );
      it = marker_names_.erase(it);
    }
  }

  server->applyChanges();
}

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "block_manipulation");
  ros::NodeHandle nh;

  tf_listener_ = new tf::TransformListener();
 
  // create marker server
  markers_ = 0;
  last_block_ = std::string("");
  server.reset( new interactive_markers::InteractiveMarkerServer("block_controls","",false) );

  ros::Duration(0.1).sleep();

  // open gripper
  client = nh.serviceClient<simple_arm_server::MoveArm>("simple_arm_server/move");
  simple_arm_server::MoveArm srv;
  simple_arm_server::ArmAction * grip = new simple_arm_server::ArmAction();
  grip->type = simple_arm_server::ArmAction::MOVE_GRIPPER;
  grip->command = 0.04;
  srv.request.goals.push_back(*grip);
  srv.request.header.frame_id="base_link";
  client.call(srv);

  // subscribe to point cloud
  ros::Subscriber s = nh.subscribe("/camera/rgb/points", 1, cloudCb);
  pub_ = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("output", 1);

  server->applyChanges();

  // everything is done in cloud callback, just spin
  ros::spin();
  
  server.reset();  
}



