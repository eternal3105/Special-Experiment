#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>

using namespace std;

/*
this is the final virsion
this node is used to merge 5 cameras' frames into one frame
pub this frame to merged_frame_topic
*/

#define rate 30
#define desired_width 480
#define desired_height 360

int main(int argc, char** argv) {
  ros::init(argc, argv, "merged_frame_publisher");
  ros::NodeHandle nh;
  // publish topic
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("merged_frame_topic", 0);
  // open 5 cameras
  cv::VideoCapture caps[] = {cv::VideoCapture("/dev/video0"),
                             cv::VideoCapture("/dev/video2"),
                             cv::VideoCapture("/dev/video4"),
                             cv::VideoCapture("/dev/video6"),
                             cv::VideoCapture("/dev/video8")};

  for (int i = 0; i < 5; ++i) {
  caps[i].set(cv::CAP_PROP_FRAME_WIDTH, desired_width);
  caps[i].set(cv::CAP_PROP_FRAME_HEIGHT, desired_height);
  }

  cv::Mat frames[5];

  // Read the first frame to determine the dimensions
  if (!caps[0].read(frames[0])) {
    ROS_ERROR("Failed to read frame from camera 0");
    return -1;
  }

  cv::Size frame_size = frames[0].size();
  cv::Mat merged_frame(frame_size.height, frame_size.width * 5, frames[0].type());
  cout << "height: " << frame_size.height << endl;
  cout << "width: " << frame_size.width << endl;
  ros::Rate loop_rate(rate);
  while (ros::ok()) {
    // Copy the first frame that was already read
    frames[0].copyTo(merged_frame(cv::Rect(0, 0, frame_size.width, frame_size.height)));

    // Read and copy the rest of the frames
    for (int i = 0; i < 5; ++i) {
      if (caps[i].read(frames[i])) {
        frames[i].copyTo(merged_frame(cv::Rect(i * frame_size.width, 0, frame_size.width, frame_size.height)));
      } else {
        ROS_ERROR("Failed to read frame from camera %d", i);
      }
    }

    sensor_msgs::ImagePtr merged_frame_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", merged_frame).toImageMsg();
    pub.publish(merged_frame_msg);
    loop_rate.sleep();
  }

  for (int i = 0; i < 5; ++i) {
    caps[i].release();
  }

  return 0;
}
