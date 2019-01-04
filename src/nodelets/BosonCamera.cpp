/*
 * Copyright © 2019 AutonomouStuff, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the “Software”), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <pluginlib/class_list_macros.h>
#include "flir_boson_usb/BosonCamera.h"

PLUGINLIB_EXPORT_CLASS(flir_boson_usb::BosonCamera, nodelet::Nodelet)

using namespace cv;
using namespace flir_boson_usb;

void BosonCamera::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  bool exit = false;
  std::string video_mode_str, zoom_enable_str, sensor_type_str;

  pnh.param<std::string>("dev_path", dev_path, "/dev/video0");
  pnh.param<std::string>("video_mode", video_mode_str, "RAW16");
  pnh.param<std::string>("zoon_enable", zoom_enable_str, "FALSE");
  pnh.param<std::string>("sensor_type", sensor_type_str, "Boson_640");

  ROS_INFO("flir_boson_usb - Got dev: %s.", dev_path.c_str());
  ROS_INFO("flir_boson_usb - Got video mode: %s.", video_mode_str.c_str());
  ROS_INFO("flir_boson_usb - Got zoom enable: %s.", zoom_enable_str.c_str());
  ROS_INFO("flir_boson_usb - Got sensor type: %s.", sensor_type_str.c_str());

  if (video_mode_str == "RAW16")
  {
    video_mode = RAW16;
  }
  else if (video_mode_str == "YUV")
  {
    video_mode = YUV;
  }
  else
  {
    exit = true;
    ROS_ERROR("flir_boson_usb - Invalid video_mode value provided. Exiting.");
  }

  if (zoom_enable_str == "TRUE" ||
      zoom_enable_str == "True" ||
      zoom_enable_str == "true")
  {
    zoom_enable = 1;
  }
  else if (zoom_enable_str == "FALSE" ||
           zoom_enable_str == "False" ||
           zoom_enable_str == "false")
  {
    zoom_enable = 0;
  }
  else
  {
    exit = true;
    ROS_ERROR("flir_boson_usb - Invalid zoom_enable value provided. Exiting.");
  }

  if (sensor_type_str == "Boson_320" ||
      sensor_type_str == "boson_320")
  {
    sensor_type = Boson320;
  }
  else if (sensor_type_str == "Boson_640" ||
           sensor_type_str == "boson_640")
  {
    sensor_type = Boson640;
  }
  else
  {
    exit = true;
    ROS_ERROR("flir_boson_usb - Invalid sensor_type value provided. Exiting.");
  }

  if (exit)
  {
    ros::shutdown();
    return;
  }

  image_transport::ImageTransport it(nh);
  pub = it.advertise("image", 10);
}

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void BosonCamera::agcBasicLinear(const Mat& input_16,
                                 Mat output_8,
                                 const int& height,
                                 const int& width)
{
  int i, j;  // aux variables

  // auxiliary variables for AGC calcultion
  unsigned int max1 = 0;         // 16 bits
  unsigned int min1 = 0xFFFF;    // 16 bits
  unsigned int value1, value2, value3, value4;

  // RUN a super basic AGC
  for (i = 0; i < height; i++)
  {
    for (j = 0; j < width; j++)
    {
      value1 = input_16.at<uchar>(i, j * 2 + 1) & 0xFF;  // High Byte
      value2 = input_16.at<uchar>(i, j * 2) & 0xFF;      // Low Byte
      value3 = (value1 << 8) + value2;

      if (value3 <= min1)
        min1 = value3;

      if (value3 >= max1)
        max1 = value3;
    }
  }

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      value1 = input_16.at<uchar>(i, j * 2 + 1) & 0xFF;     // High Byte
      value2 = input_16.at<uchar>(i, j * 2) & 0xFF;         // Low Byte
      value3 = (value1 << 8) + value2;
      value4 = ((255 * (value3 - min1))) / (max1 - min1);

      output_8.at<uchar>(i, j) = static_cast<uint8_t>(value4 & 0xFF);
    }
  }
}
