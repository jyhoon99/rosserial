#include <ros/ros.h>
#include <chrono>
#include <thread>

#include <std_msgs/String.h>
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <hatch/Message.h>


using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION 1.0

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL1 ID
#define DXL3_ID               3            // DXL2 ID
#define BAUDRATE              115200          // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler;
PacketHandler * packetHandler;

void topicCallback(const hatch::Message::ConstPtr &work)
{
  uint8_t mod = (uint8_t)work->mode;
  uint8_t dxl_error = 0; 
  int dxl_comm_result = 0;

  if (mod == 1){
    //동작1
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, 6554000, &dxl_error);
    sleep(6); // wait for 10 seconds
    //리니어
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, 6553800, &dxl_error);
    sleep(6);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, 6554150, &dxl_error);
  } else if (mod == 2){
    //동작2
   dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, 6554100, &dxl_error);    //655855
  sleep(3); // wait for 5 seconds
   dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_POSITION, 6553650, &dxl_error);    //655855
  sleep(3);
   dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_POSITION, 6553800, &dxl_error);    //655855    
  } else if (mod == 3){
    //동작3
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, 6553790, &dxl_error);    
  }
    ROS_INFO("Received message:");
    ROS_INFO("mode = %d", work->mode);
}

int main(int argc, char** argv)
{
   uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }
  
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("JYH Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("JYH Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("CSM Failed to enable torque for Dynamixel ID %d", DXL3_ID);
    return -1;
  }

  ros::init(argc, argv, "back_sub_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/topic", 10, topicCallback);
  ros::spin();

  portHandler->closePort();
  return 0;


}