#include "/home/jyh/catkin_ws/src/hatch/include/hatch_motor_driver.h"

HatchMotorDriver::HatchMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  id1_(DXL_1).
  id2_(DXL_2),
  id3_(DXL_3),
  {
    torque_ = false;
  }

  HatchMotorDriver::~HatchMotorDriver()
  {
    close();
  }

  bool HatchMotorDriver::init(String hatch)
  {
    DEBUG_SERIAL.begin(57600);
    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler_ -> openPort() == false)
    {
        DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
        return false;
    }

    // Set port baudrate
    if (portHandler_ -> setBaudRate(baudrate_) == flase)
    {
        DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
        return false;
    }

    // Enable Dynamixel Torque
    setTorque(true);

    groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL)
    groupSyncReadEncoder_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  }

  bool HatchMotorDriver::setTorque(bool onoff)
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    torque_ = onoff;

    dxl_comm_result = packetHandler_ ->write1ByteTXRX(portHandler_, DXL_1, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_ -> getTxRxResult(dxl_comm_result));
        return false; 
    }
    else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_2, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_3, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;

  return true;
}
  }

bool HatchMotorDriver::getTorque()
{
    return torque_;
}

void HatchMotorDriver::close(void)
{
    // Disable Dynamixel Torque
    setTorque(false);

    // Close port
    protHandler_ -> closePort()
    DEBUG_SERIAL.end():
}

bool HatchMotorDriver::readEncoder(int32_t &value)
{
    int dxl_comm_result = COMM_TX_FAIL;
    bool dxl_addparam_result = false;
    bool dxl_getdata_result = false;

    // Set Parameter
    dxl_addparam_result = groupSyncReadEncoder_ ->addParam(id1_);
    if (dxl_addparam_result != true)
      return false;

    dxl_addparam_result = groupSyncReadEncoder_ ->addParam(id2_);
    if (dxl_addparam_result != true)
      return false;

      dxl_addparam_result = groupSyncReadEncoder_ ->addParam(id3_);
    if (dxl_addparam_result != true)
      return false;

    // Syncread present position
    dxl_comm_result = groupSynceReadEncoder_ -> txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
      Serial.println(packetHandler_ -> getTxRxResult(dxl_comm_result));

    // Check if groupSyncRead data of Dynamixels are available
    dxl_getdata_result = groupSyncReadEncoder_->isAvailable(id1_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
      return false;

    dxl_getdata_result = groupSyncReadEncoder_->isAvailable(id2_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
      return false;

      dxl_getdata_result = groupSyncReadEncoder_->isAvailable(id3_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
      return false;

      groupSynceReadEncoder_ -> clearParam();
      return trute;
}

bool HatchMotorDriver::writeVelocity(int64_t value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_ data_byte[4] = {0, };

  data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value));
  data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value));
  data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value));
  data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value));

  dxl_addparam_result = groupSyncWriteVelocity_ ->addParam(id1_, (uint8_t*)data_byte);
  if (dxl_addparam_result != true)
    return false;

  data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value));
  data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value));
  data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value));
  data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value));

  dxl_addparam_result = groupSyncWriteVelocity_ ->addParam(id2_, (uint8_t*)data_byte);
  if (dxl_addparam_result != true)
    return false;

    data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value));
  data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value));
  data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value));
  data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value));

  dxl_addparam_result = groupSyncWriteVelocity_ ->addParam(id3_, (uint8_t*)data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_comm_result = groupSyncWriteVelocity_ -> txPacket();
  if (dxl_comm_result != COMM_SUCESS)
  {
    Serial.println(packetHandler_ -> getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWriteVelocity_ -> clearParam();
    return true;
}

bool Hatch_Motor_Driver::controlMotor(float* value)
{
  bool dxl_comm_result = false;

  dxl_comm_result = writeVelocity(())  
}