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

// inputs, cloud & pixel
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

// outputs
#include <simple_grasping/PickAction.h>

// this module converts a point request into grasp request
//
// inputs:  point cloud
//          (x,y) pixel to grasp
//
// outputs: creates a bounding box for object to grasp, adds to collision environment
//          action request to grasp server

class GraspPerception
{
  private:


  public:
    GraspPerception(){

        // subscribe to just the cloud now
        cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &ChessBoardLocator::cameraCallback, this);
        cloud_pub_ = nh_.advertise< pcl::PointCloud<point> >("cloud", 1);
    }

    
    void cloudCallback()
    {

    }

    void pointCallback()
    {

    }

  private: 
    /* node handles, subscribers, publishers, etc */
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;

    pcl::RadiusOutlierRemoval<point> radius_removal_;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_grasp_perception");
  ros::NodeHandle nh;
  GraspServer server(nh);
  ros::spin();
  return 0;
}

