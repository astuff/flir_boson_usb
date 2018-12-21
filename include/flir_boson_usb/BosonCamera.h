#ifndef FLIR_BOSON_USB_BOSON_CAMERA_H
#define FLIR_BOSON_USB_BOSON_CAMERA_H

// C++ Includes
#include <string>

// Linux system includes
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

// OpenCV Includes
#include <opencv2/opencv.hpp>

// ROS Includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace flir_boson_usb
{

enum Encoding
{
  YUV = 0,
  RAW16 = 1
};

enum SensorTypes
{
  Boson320,
  Boson640
};

class BosonCamera : public nodelet::Nodelet
{
  public:
    BosonCamera() {};
    ~BosonCamera() {};

  private:
    virtual void onInit();
    void agc_basic_linear(const cv::Mat& input_16,
                          cv::Mat output_8,
                          const int& height,
                          const int& width);

    int width, height;
    image_transport::Publisher pub;
};

}  // namespace flir_boson_usb

#endif  // FLIR_BOSON_USB_BOSON_CAMERA_H
