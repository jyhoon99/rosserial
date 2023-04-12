#include <ros/ros.h>
#include <std_msgs/String.h>
//  #include <DynamixelSDK.h>
// #include "dynamixel_sdk_examples/GetPosition.h"
// #include "dynamixel_sdk_examples/SetPosition.h"


void frontCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data == "f")
   {
      ROS_INFO("Received message: %s", msg->data.c_str());
    // 전면부 기능 실행 코드

  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "front_sub_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("hatch_topic", 1000, frontCallback);

  ros::spin();

  return 0;
}