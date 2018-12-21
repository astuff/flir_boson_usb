#include <pluginlib/class_list_macros.h>
#include "flir_boson_usb/BosonCamera.h"

PLUGINLIB_EXPORT_CLASS(flir_boson_usb::BosonCamera, nodelet::Nodelet)

using namespace cv;
using namespace flir_boson_usb;

void BosonCamera::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  image_transport::ImageTransport it(nh);
  pub = it.advertise("image", 10);
}

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void BosonCamera::agc_basic_linear(const Mat& input_16,
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
