#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <sstream>

using namespace std;

static int image_width;
static int image_height;
static int fps;

string video_device;
string remap_device;

int main(int argc, char** argv) {
  ros::init(argc, argv, "merged_frame_publisher");
  ros::NodeHandle nh("~");
  

  nh.param<string>("video_device", video_device, "/dev/video0");
  nh.param<int>("image_width", image_width, 352);
  nh.param<int>("image_height", image_height, 288);
  nh.param<int>("fps", fps, 30);
  nh.param<string>("remap_device", remap_device, "/camera_0");

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>(remap_device, 1);
  
  cv::VideoCapture cap = cv::VideoCapture(video_device);

  cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);

  cv::Mat frame;
  sensor_msgs::ImagePtr frame_msg(new sensor_msgs::Image);
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "bgr8";

  ros::Rate loop_rate(fps);
  while (ros::ok()) {
    if (!cap.read(frame)) {
      ROS_ERROR("Failed to read frame from %s", remap_device.c_str());
      return -1;
    }
    cv_image.image = frame;
    cv_image.toImageMsg(*frame_msg);
    pub.publish(frame_msg);
    loop_rate.sleep();
  }

  cap.release();

  return 0;
}