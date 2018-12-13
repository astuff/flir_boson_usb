// C++ Includes
#include <cstdio>
#include <fstream>
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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

using namespace cv;

enum Encoding {
  YUV = 0,
  RAW16 = 1
};

enum SensorTypes {
  Boson320,
  Boson640
};

int width;
int height;

/* ---------------------------- 16 bits Mode auxiliary functions ---------------------------------------*/

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void AGC_Basic_Linear(Mat input_16, Mat output_8, int height, int width)
{
  int i, j;  // aux variables

  // auxiliary variables for AGC calcultion
  unsigned int max1 = 0;         // 16 bits
  unsigned int min1 = 0xFFFF;    // 16 bits
  unsigned int value1, value2, value3, value4;

  // RUN a super basic AGC
  for (i=0; i<height; i++)
  {
    for (j=0; j<width; j++)
    {
      value1 = input_16.at<uchar>(i,j*2+1) & 0XFF;  // High Byte
      value2 = input_16.at<uchar>(i,j*2) & 0xFF;    // Low Byte
      value3 = ( value1 << 8) + value2;

      if (value3 <= min1)
      {
        min1 = value3;
      }

      if (value3 >= max1)
      {
        max1 = value3;
      }
    }
  }

  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      value1 = input_16.at<uchar>(i,j*2+1) & 0XFF;  // High Byte
      value2 = input_16.at<uchar>(i,j*2) & 0xFF;    // Low Byte
      value3 = ( value1 << 8) + value2;
      value4 = ( ( 255 * ( value3 - min1) ) ) / (max1-min1);

      output_8.at<uchar>(i,j) = (uchar)(value4 & 0xFF);
    }
  }
}

int main(int argc, char** argv)
{
  int ret;
  int fd;
  int i;
  struct v4l2_capability cap;
  long frame = 0;        // First frame number enumeration
  char label[50];        // To display the information
  char thermal_sensor_name[20];  // To store the sensor name
  // Default Program options
  std::string dev_path = "/dev/video0";
  Encoding video_mode = RAW16;
  int zoom_enable = 0;
  SensorTypes sensor_type = Boson320;

  // Set up ROS
  ros::init(argc, argv, "flir_boson_usb_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Get node parameters
  bool exit = false;
  std::string video_mode_str = "RAW16";
  std::string zoom_enable_str = "false";
  std::string sensor_type_str = "Boson_320";

  pnh.getParam("dev", dev_path);
  ROS_INFO("flir_boson_usb - Got dev: %s.", dev_path.c_str());

  if (pnh.getParam("video_mode", video_mode_str))
  {
    if (video_mode_str == "RAW16")
    {
      video_mode = RAW16;
      ROS_INFO("flir_boson_usb - Got video_mode: RAW16.");
    }
    else if (video_mode_str == "YUV")
    {
      video_mode = YUV;
      ROS_INFO("flir_boson_usb - Got video_mode: YUV.");
    }
    else
    {
      exit = true;
      ROS_ERROR("flir_boson_usb - Got invalid value for video_mode. Exiting.");
    }
  }

  if (pnh.getParam("zoom_enable", zoom_enable_str))
  {
    if (zoom_enable_str == "TRUE" ||
        zoom_enable_str == "True" ||
        zoom_enable_str == "true")
    {
      zoom_enable = 1;
      ROS_INFO("flir_boson_usb - Got zoom_enable: true.");
    }
    else if (zoom_enable_str == "FALSE" ||
             zoom_enable_str == "False" ||
             zoom_enable_str == "false")
    {
      zoom_enable = 0;
      ROS_INFO("flir_boson_usb - Got zoom_enable: false.");
    }
    else
    {
      exit = true;
      ROS_ERROR("flir_boson_usb - Got invalid value for zoom_enable. Exiting.");
    }
  }

  if (pnh.getParam("sensor_type", sensor_type_str))
  {
    if (sensor_type_str == "Boson_320" ||
        sensor_type_str == "boson_320")
    {
      sensor_type = Boson320;
      ROS_INFO("flir_boson_usb - Got sensor_type: Boson_320.");
    }
    else if (sensor_type_str == "Boson_640" ||
             sensor_type_str == "boson_640")
    {
      sensor_type = Boson640;
      ROS_INFO("flir_boson_usb - Got sensor_type: Boson_640.");
    }
    else
    {
      exit = true;
      ROS_ERROR("flir_boson_usb - Got invalid value for sensor_type. Exiting.");
    }
  }

  if (exit)
    ros::shutdown();

  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("image", 10);

  cv_bridge::CvImage cv_img;
  sensor_msgs::ImagePtr pub_image;

  // Open the Video device
  if ((fd = open(dev_path.c_str(), O_RDWR)) < 0)
  {
    ROS_ERROR("flir_boson_usb - ERROR : OPEN. Invalid Video Device.");
    ros::shutdown();
    return -1;
  }

  // Check VideoCapture mode is available
  if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
  {
    ROS_ERROR("flir_boson_usb - ERROR : VIDIOC_QUERYCAP. Video Capture is not available.");
    ros::shutdown();
    return -1;
  }

  if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
    ROS_ERROR("flir_boson_usb - The device does not handle single-planar video capture.");
    ros::shutdown();
    return -1;
  }

	struct v4l2_format format;

  // Two different FORMAT modes, 8 bits vs RAW16
  if (video_mode == RAW16) {
    // I am requiring thermal 16 bits mode
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;

    // Select the frame SIZE (will depend on the type of sensor)
    switch (sensor_type)
    {
      case Boson320:  // Boson320
        width = 320;
        height = 256;
        break;
      case Boson640:  // Boson640
        width = 640;
        height = 512;
        break;
      default:  // Boson320
        width = 320;
        height = 256;
        break;
    }
  }
  else // 8- bits is always 640x512 (even for a Boson 320)
  {
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420; // thermal, works   LUMA, full Cr, full Cb
    width = 640;
    height = 512;
  }

  // Common varibles
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = width;
  format.fmt.pix.height = height;

  // request desired FORMAT
  if (ioctl(fd, VIDIOC_S_FMT, &format) < 0)
  {
    ROS_ERROR("flir_boson_usb - VIDIOC_S_FMT error.");
    ros::shutdown();
    return -1;
  }

  // we need to inform the device about buffers to use.
  // and we need to allocate them.
  // weâ€™ll use a single buffer, and map our memory using mmap.
  // All this information is sent using the VIDIOC_REQBUFS call and a
  // v4l2_requestbuffers structure:
  struct v4l2_requestbuffers bufrequest;
  bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufrequest.memory = V4L2_MEMORY_MMAP;
  bufrequest.count = 1;   // we are asking for one buffer

  if (ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0)
  {
    ROS_ERROR("flir_boson_usb - VIDIOC_REQBUFS error.");
    ros::shutdown();
    return -1;
  }

  // Now that the device knows how to provide its data,
  // we need to ask it about the amount of memory it needs,
  // and allocate it. This information is retrieved using the VIDIOC_QUERYBUF call,
  // and its v4l2_buffer structure.

  struct v4l2_buffer bufferinfo;
  memset(&bufferinfo, 0, sizeof(bufferinfo));

  bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo.memory = V4L2_MEMORY_MMAP;
  bufferinfo.index = 0;

  if (ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0)
  {
    ROS_ERROR("flir_boson_usb - VIDIOC_QUERYBUF error.");
    ros::shutdown();
    return -1;
  }

  // map fd+offset into a process location (kernel will decide due to our NULL). length and
  // properties are also passed
  void * buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, bufferinfo.m.offset);

  if (buffer_start == MAP_FAILED)
  {
    ROS_ERROR("flir_boson_usb - mmap error.");
    ros::shutdown();
    return -1;
  }

  // Fill this buffer with ceros. Initialization. Optional but nice to do
  memset(buffer_start, 0, bufferinfo.length);

  // Activate streaming
  int type = bufferinfo.type;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
  {
    ROS_ERROR("flir_boson_usb - VIDIOC_STREAMON error.");
    ros::shutdown();
    return -1;
  }

  // Declarations for RAW16 representation
  // Will be used in case we are reading RAW16 format
  // Boson320 , Boson 640
  Mat thermal16(height, width, CV_16U, buffer_start);   // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
  Mat thermal16_linear(height,width, CV_8U, 1);         // OpenCV output buffer : Data used to display the video

  // Declarations for Zoom representation
  // Will be used or not depending on program arguments
  Size size(640,512);
  Mat thermal16_linear_zoom;   // (height,width, CV_8U, 1);    // Final representation
  Mat thermal_rgb_zoom;   // (height,width, CV_8U, 1);    // Final representation

  int luma_height ;
  int luma_width ;
  int color_space ;;

  // Declarations for 8bits YCbCr mode
  // Will be used in case we are reading YUV format
  // Boson320, 640 :  4:2:0
  luma_height = height+height/2;
  luma_width = width;
  color_space = CV_8UC1;
  Mat thermal_luma(luma_height, luma_width,  color_space, buffer_start);  // OpenCV input buffer
  Mat thermal_rgb(height, width, CV_8UC3, 1);    // OpenCV output buffer , BGR -> Three color spaces (640 - 640 - 640 : p11 p21 p31 .... / p12 p22 p32 ..../ p13 p23 p33 ...)

  // Read frame, do AGC, paint frame
  while (ros::ok())
  {
    // Put the buffer in the incoming queue.
    if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0)
    {
      ROS_ERROR("flir_boson_usb - VIDIOC_QBUF error.");
      ros::shutdown();
      return -1;
    }

    // The buffer's waiting in the outgoing queue.
    if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0)
    {
      ROS_ERROR("flir_boson_usb - VIDIOC_QBUF");
      ros::shutdown();
      return -1;
    }

    if (video_mode == RAW16)
    {
      // -----------------------------
      // RAW16 DATA
      AGC_Basic_Linear(thermal16, thermal16_linear, height, width);

      // Display thermal after 16-bits AGC... will display an image
      if (zoom_enable == 0)
      {
        // Threshold using Otsu's method, then use the result as a mask on the original image
        Mat mask_mat, masked_img;
        threshold(thermal16_linear, mask_mat, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
        thermal16_linear.copyTo(masked_img, mask_mat);

        // Normalize the pixel values to the range [0, 1] then raise to power (gamma). Then convert back for display.
        Mat d_out_img, norm_image, d_norm_image, gamma_corrected_image, d_gamma_corrected_image;
        double gamma = 0.8;
        masked_img.convertTo(d_out_img, CV_64FC1);
        normalize(d_out_img, d_norm_image, 0, 1, NORM_MINMAX, CV_64FC1);
        pow(d_out_img, gamma, d_gamma_corrected_image);
        d_gamma_corrected_image.convertTo(gamma_corrected_image, CV_8UC1);
        normalize(gamma_corrected_image, gamma_corrected_image, 0, 255, NORM_MINMAX, CV_8UC1);

        // Apply top hat filter
        int erosion_size = 5;
        Mat top_hat_img, kernel = getStructuringElement(MORPH_ELLIPSE, Size(2 * erosion_size + 1, 2 * erosion_size + 1));
        morphologyEx(gamma_corrected_image, top_hat_img, MORPH_TOPHAT, kernel);

        cv_img.image = thermal16_linear;
        cv_img.header.stamp = ros::Time::now();
        cv_img.header.frame_id = "boson_camera";
        cv_img.encoding = "mono8";
        pub_image = cv_img.toImageMsg();
        image_pub.publish(pub_image);
      }
      else
      {
        resize(thermal16_linear, thermal16_linear_zoom, size);

        cv_img.image = thermal16_linear_zoom;
        cv_img.header.stamp = ros::Time::now();
        cv_img.header.frame_id = "boson_camera";
        cv_img.encoding = "mono8";
        pub_image = cv_img.toImageMsg();
        image_pub.publish(pub_image);
      }
    }
    else // Video is in 8 bits YUV
    {
      // ---------------------------------
      // DATA in YUV
      cvtColor(thermal_luma, thermal_rgb, COLOR_YUV2GRAY_I420, 0 );    
      cv_img.image = thermal_rgb;
      cv_img.encoding = "mono8";
      
      cv_img.header.stamp = ros::Time::now();
      cv_img.header.frame_id = "boson_camera";
      pub_image = cv_img.toImageMsg();
      image_pub.publish(pub_image);
    }

    ros::spinOnce();
  }

  // Finish loop. Exiting.
	// Deactivate streaming
	if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0 )
  {
		ROS_ERROR("flir_boson_usb - VIDIOC_STREAMOFF error.");
		ros::shutdown();
    return -1;
	};

	close(fd);

  return 0;
}
