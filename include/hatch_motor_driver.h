#ifndef HATCH_MOTOR_DRIVER_H_
#define HATCH_MOTOR_DRIVER_H_

#include "vriant.h"
#include <DynamixelSDK.h>

///////////////////////////////////////////////////////////////////////
// Control table address (Dynamixel AX-series 1.0)
/////////////////////////////////////////////////////////////////////////
#define ADDR_X_TORQUE_ENABLE         24
#define ADDR_X_GOAL POSITION         30
#define ADDR_X_MOVNG SPEED           32
#define ADDR_X_PRESENT_POSITION      40
#define ADDR_X_PRESENT_SPEED         38

///////////////////////////////////////////////////////////////////////
// DATA Byte Length (Dynamixel AX-series 1.0)
/////////////////////////////////////////////////////////////////////////
#define LEN_X_TORQUE_ENABLE         1
#define LEN_X_GOAL_POSITION         2
#define LEN_X_MOVING SPEED          2
#define LEN_X_PRESENT_POSITION      2
#define LEN_X_PRESENT_SPEED         2

///////////////////////////////////////////////////////////////////////

#define PROTOCOL_VERSION        1.0
#define DXL_1                   1
#define DXL_2                   2
#define DXL_3                   3

#define BAUDRATE                115200
#define DEVICENAME              ""

#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0

#define DEBUG_SERIAL SerialBT2

class HatchMotorDriver
{
    public:
    HatchMotorDriver();
    ~HatchMotorDriver();
    bool init(String hatch);
    void close(void);
    bool setTorque(bool onoff);
    bool getTorque();
    bool readEncoder(int32_t &value);
    bool writeVelocity(int64_t value);
    bool contorlMotor(float* velocity);

    private:
    uint32_t baudrate_;
    float protocol_version_;
    uint8_t id1_;
    uint8_t id2_;
    uint8_t id3_;
    bool torque_;

    dynamixel::PortHandler  *portHandler_;
    dynamixel::PacketHandler    *pakcetHandler_;

    dynamixel::GroupSyncWrite   *groupSyncWriteVelocity_;
    dynamixel::GroupSyncRead    *groupSyncReadEncoder_;
};

#endif      //HATCH_MOTOR_DRIVER_H