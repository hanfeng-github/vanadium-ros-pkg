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
#ifndef USB_CAM_USB_CAM_H
#define USB_CAM_USB_CAM_H

#include <string>
#include <sstream>

extern "C" {
#include <linux/videodev2.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

typedef struct {
  int width;
  int height;
  int bytes_per_pixel;
  int image_size;
  char *image;
  int is_new;
} usb_cam_camera_image_t;

struct buffer {
  void * start;
  size_t length;
};

typedef enum {
  IO_METHOD_READ,
  IO_METHOD_MMAP,
  IO_METHOD_USERPTR,
} usb_cam_io_method;

typedef enum {
  PIXEL_FORMAT_YUYV,
  PIXEL_FORMAT_UYVY,
  PIXEL_FORMAT_MJPEG,
} usb_cam_pixel_format;

class UsbCamera{
  int fd;
  char* camera_dev;
  usb_cam_io_method io;
  usb_cam_pixel_format pf;
  int w;
  int h;
  unsigned int pixelformat;

  struct buffer * buffers;
  unsigned int n_buffers;

  AVFrame *avframe_camera;
  AVFrame *avframe_rgb;
  AVCodec *avcodec;
  AVCodecContext *avcodec_context;
  int avframe_camera_size;
  int avframe_rgb_size;

  struct SwsContext *video_sws;

  int read_frame(usb_cam_camera_image_t *image);
  void init_read(unsigned int buffer_size);
  void init_mmap(void);
  void init_userp(unsigned int buffer_size);
  void start_capturing();
  void init_device(int, int);
  void open_device(void);

  void stop_capturing(void);
  void uninit_device(void);
  void close_device(void);

  int init_mjpeg_decoder(int, int);
  void mjpeg2rgb(char*, int, char*, int);
  void process_image(const void * src, int len, usb_cam_camera_image_t *dest);

public:
  // start camera
  UsbCamera(const char* dev, usb_cam_io_method io, usb_cam_pixel_format pf, int image_width, int image_height, usb_cam_camera_image_t * image);
  // shutdown camera
  void shutdown();
  // grabs a new image from the camera
  void grab(usb_cam_camera_image_t *image);
};

#endif

