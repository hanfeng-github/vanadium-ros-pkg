/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <stdio.h>
#include <iostream>
/* 
 * Modified for stereo imaging
 * Michael E. Ferguson, 2010
 */
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <new_usb_cam/new_usb_cam.h>
#include <self_test/self_test.h>
#include <image_transport/image_transport.h>

class UsbCamNode
{
public:
  ros::NodeHandle node_;

  std::string io_method_name_;
  int image_width_,image_height_;
  std::string pixel_format_name_;

  /* left side */
  sensor_msgs::Image left_img_;
  std::string left_video_device_name_;
  sensor_msgs::CameraInfo left_info_;
  usb_cam_camera_image_t* left_camera_image_;
  image_transport::CameraPublisher left_image_pub_;
  UsbCamera * left;
  
  /* right side */
  sensor_msgs::Image right_img_;
  std::string right_video_device_name_;
  sensor_msgs::CameraInfo right_info_;
  usb_cam_camera_image_t* right_camera_image_;
  image_transport::CameraPublisher right_image_pub_;
  UsbCamera * right;

  ros::Time next_time_;
  int count_;
  
  UsbCamNode() :
      node_("~")
  {
    image_transport::ImageTransport it(node_);
    left_image_pub_ = it.advertiseCamera("left/image_raw", 1);
    right_image_pub_ = it.advertiseCamera("right/image_raw", 1);

    // left 
    node_.param("left/video_device", left_video_device_name_, std::string("/dev/video0"));
    node_.param("right/video_device", right_video_device_name_, std::string("/dev/video1"));
    node_.param("io_method", io_method_name_, std::string("mmap")); // possible values: mmap, read, userptr
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg")); // possible values: yuyv, uyvy, mjpeg

    {
      XmlRpc::XmlRpcValue double_list;

      /* Left Camera Setup */
      left_info_.height = image_height_;
      left_info_.width = image_width_;
      node_.param("left/camera_frame_id", left_img_.header.frame_id, std::string("left_head_camera")); 
      left_info_.header.frame_id = left_img_.header.frame_id;
      
      node_.getParam("left/K", double_list);
      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          left_info_.K[i] = double_list[i];
        }
      }

      node_.getParam("left/D", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 5)) {
        for (int i=0; i<5; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          left_info_.D[i] = double_list[i];
        }
      }

      node_.getParam("left/R", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          left_info_.R[i] = double_list[i];
        }
      }

      node_.getParam("left/P", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 12)) {
        for (int i=0; i<12; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          left_info_.P[i] = double_list[i];
        }
      }
    
      // Right camera setup
      right_info_.height = image_height_;
      right_info_.width = image_width_;
      node_.param("right/camera_frame_id", right_img_.header.frame_id, std::string("right_head_camera")); // TODO? 
      right_info_.header.frame_id = right_img_.header.frame_id;

      node_.getParam("right/K", double_list);
      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          right_info_.K[i] = double_list[i];
        }
      }

      node_.getParam("right/D", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 5)) {
        for (int i=0; i<5; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          right_info_.D[i] = double_list[i];
        }
      }

      node_.getParam("right/R", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          right_info_.R[i] = double_list[i];
        }
      }

      node_.getParam("right/P", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 12)) {
        for (int i=0; i<12; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          right_info_.P[i] = double_list[i];
        }
      }

    }

    printf("usb_cam left/video_device set to [%s]\n", left_video_device_name_.c_str());
    printf("usb_cam right/video_device set to [%s]\n", right_video_device_name_.c_str());
    printf("usb_cam io_method set to [%s]\n", io_method_name_.c_str());
    printf("usb_cam image_width set to [%d]\n", image_width_);
    printf("usb_cam image_height set to [%d]\n", image_height_);
    printf("usb_cam pixel_format set to [%s]\n", pixel_format_name_.c_str());

    usb_cam_io_method io_method;
    if(io_method_name_ == "mmap")
      io_method = IO_METHOD_MMAP;
    else if(io_method_name_ == "read")
      io_method = IO_METHOD_READ;
    else if(io_method_name_ == "userptr")
      io_method = IO_METHOD_USERPTR;
    else {
      ROS_FATAL("Unknown io method.");
      node_.shutdown();
      return;
    }

    usb_cam_pixel_format pixel_format;
    if(pixel_format_name_ == "yuyv")
      pixel_format = PIXEL_FORMAT_YUYV;
    else if(pixel_format_name_ == "uyvy")
      pixel_format = PIXEL_FORMAT_UYVY;
    else if(pixel_format_name_ == "mjpeg") {
      pixel_format = PIXEL_FORMAT_MJPEG;
    }
    else {
      ROS_FATAL("Unknown pixel format.");
      node_.shutdown();
      return;
    }

    // initialize cameras!
    left = new UsbCamera(left_video_device_name_.c_str(),
        io_method,
        pixel_format,
        image_width_,
        image_height_,
        left_camera_image_);
        left_camera_image_ = (usb_cam_camera_image_t *) calloc(1, sizeof(usb_cam_camera_image_t));

        left_camera_image_->width = image_width_;
        left_camera_image_->height = image_height_;
        left_camera_image_->bytes_per_pixel = 24;

        left_camera_image_->image_size = left_camera_image_->width*left_camera_image_->height*left_camera_image_->bytes_per_pixel;
        left_camera_image_->is_new = 0;
        left_camera_image_->image = (char *) calloc(left_camera_image_->image_size, sizeof(char));
        memset(left_camera_image_->image, 0, left_camera_image_->image_size*sizeof(char));

    right = new UsbCamera(right_video_device_name_.c_str(),
        io_method,
        pixel_format,
        image_width_,
        image_height_,
        right_camera_image_);
        right_camera_image_ = (usb_cam_camera_image_t *) calloc(1, sizeof(usb_cam_camera_image_t));

        right_camera_image_->width = image_width_;
        right_camera_image_->height = image_height_;
        right_camera_image_->bytes_per_pixel = 24;

        right_camera_image_->image_size = right_camera_image_->width*right_camera_image_->height*right_camera_image_->bytes_per_pixel;
        right_camera_image_->is_new = 0;
        right_camera_image_->image = (char *) calloc(right_camera_image_->image_size, sizeof(char));
        memset(right_camera_image_->image, 0, right_camera_image_->image_size*sizeof(char));

    /*left_camera_image_ = usb_cam_camera_start(left_video_device_name_.c_str(),
        io_method,
        pixel_format,
        image_width_,
        image_height_);
    right_camera_image_ = usb_cam_camera_start(right_video_device_name_.c_str(),
        io_method,
        pixel_format,
        image_width_,
        image_height_);*/

    next_time_ = ros::Time::now();
    count_ = 0;
  }

  virtual ~UsbCamNode()
  {
    left->shutdown();
    right->shutdown();
  }

  bool take_and_send_image()
  {
    left->grab(left_camera_image_);
    fillImage(left_img_, "rgb8", left_camera_image_->height, left_camera_image_->width, 3 * left_camera_image_->width, left_camera_image_->image);
    right->grab(right_camera_image_);
    fillImage(right_img_, "rgb8", right_camera_image_->height, right_camera_image_->width, 3 * right_camera_image_->width, right_camera_image_->image);

    left_img_.header.stamp = ros::Time::now();
    right_img_.header.stamp = left_img_.header.stamp;
    left_info_.header.stamp = left_img_.header.stamp;
    right_info_.header.stamp = right_img_.header.stamp;
    left_image_pub_.publish(left_img_, left_info_);
    right_image_pub_.publish(right_img_, right_info_);
    return true;
  }


  bool spin()
  {
    while (node_.ok())
    {
      if (take_and_send_image())
      {
        count_++;
        ros::Time now_time = ros::Time::now();
        if (now_time > next_time_) {
          std::cout << count_ << " frames/sec at " << now_time << std::endl;
          count_ = 0;
          next_time_ = next_time_ + ros::Duration(1,0);
        }
      } else {
        ROS_ERROR("couldn't take image.");
        usleep(1000000);
      }
//      self_test_.checkTest();
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_usb_cam");
  UsbCamNode a;
  a.spin();
  return 0;
}

