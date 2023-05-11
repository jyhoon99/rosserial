#include <ros/ros.h>
#include <std_msgs/String.h>
#include "dynamixel_sdk_examples/SetPosition.h"
#include <hatch/Message.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Publisher_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<hatch::Message>("topic",10);
  ros::Rate loop_rate(10);
  hatch::Message work;
  while (ros::ok())
  {
    // getchar() 버퍼 내용 모두 비우기
    while(std::getchar() != '\n');

    // 키보드 입력 받기
    char key = std::getchar();

    if(key == 'f'){
      work.mode = 1;
    } else if (key == 'g'){
      work.mode = 2;
    } else if (key == 'h'){
      work.mode = 3;
    } 

    pub.publish(work);
    loop_rate.sleep();
  }
  return 0;
}