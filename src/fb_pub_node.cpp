#include <ros/ros.h>
#include <std_msgs/String.h>
#include "dynamixel_sdk_examples/SetPosition.h"


int main(int argc, char** argv)
{
  // ROS node 초기화
  ros::init(argc, argv, "Publisher_node");

  // ROS node handle 생성
  ros::NodeHandle nh;

  // 토픽 이름 정의
  std::string topic_name = "set_position";

  // 퍼블리셔 생성
  ros::Publisher pub = nh.advertise<dynamixel_sdk_examples::SetPosition>(topic_name, 100);

  // 루프 주기 설정
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // 사용자로부터 입력 받기
    char key_input = std::getchar();

    // 입력된 값에 따라 메세지 생성
    dynamixel_sdk_examples::SetPosition msg;
    if (key_input == 'f') {
      msg.id = 1;
      msg.position = 400;
    } else if (key_input == 'g') {
      msg.id = 1;
      msg.position = 200;
    }
    else if (key_input == 'h') {
      msg.id = 1;
      msg.position = 550;
    }           //front-back
    else if (key_input == 'v') {
      msg.id = 2;
      msg.position = 500;
    }
    else if (key_input == 'b') {
      msg.id = 3;
      msg.position = 50;
    }
    else if (key_input == 'n') {
      msg.id = 3;
      msg.position = 200;
    }
    else if (key_input == 'm') {
      msg.id = 2;
      msg.position = 190;
    } else {
      continue;  // f나 b가 아닌 입력은 무시
    }

    // 메세지 발행
    pub.publish(msg);

    // 루프 주기 맞추기
    loop_rate.sleep();
  }

  return 0;
}