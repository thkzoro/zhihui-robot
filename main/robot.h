#ifndef ESP_IDF_V4_4_ROBOT_H
#define ESP_IDF_V4_4_ROBOT_H

#include "round_lcd.h"
#include "servo.h"
#include "paj7620.h"
#include "usb_robot.h"

#define ANY 0

class Robot {
public:
    Robot() {m_pLcd = nullptr; m_pPAJ7620 = nullptr; m_pUsbRobot = nullptr;}
    ~Robot() {if (m_pLcd) delete m_pLcd; if (m_pPAJ7620) delete m_pPAJ7620; if (m_pUsbRobot) delete m_pUsbRobot;}

    int  do_init();
    void do_poll();

    struct UsbBuffer_t {
        uint8_t extraDataTx[32];
        uint8_t rxData[2][60 * 240 * 3 + 32]; // 43232bytes, 43200 of which are lcd buffer
        volatile uint16_t receivedPacketLen = 0;
        volatile uint8_t pingPongIndex = 0;
        volatile uint32_t rxDataOffset = 0;
    };
    UsbBuffer_t usbBuffer;

    struct JointStatus_t {
        uint8_t id;
        float angleMin;
        float angleMax;
        float angle;
        float modelAngelMin;
        float modelAngelMax;
        bool inverted = false;
    };
    JointStatus_t joint[7];

    uint8_t* GetPingPongBufferPtr();
    uint8_t* GetLcdBufferPtr();
    uint8_t* GetExtraDataRxPtr();
    void SwitchPingPongBuffer();
    void SendUsbPacket(uint8_t* _data, uint32_t _len);
    void ReceiveUsbPacketUntilSizeIs(uint32_t _count);
    void SetJointId(JointStatus_t &_joint, uint8_t _id);
    void SetJointKp(JointStatus_t &_joint, float _value);
    void SetJointKi(JointStatus_t &_joint, float _value);
    void SetJointKv(JointStatus_t &_joint, float _value);
    void SetJointKd(JointStatus_t &_joint, float _value);
    void SetJointEnable(JointStatus_t &_joint, bool _enable);
    void SetJointInitAngle(JointStatus_t &_joint, float _angle);
    void SetJointTorqueLimit(JointStatus_t &_joint, float _percent);

    void UpdateServoAngle(JointStatus_t &_joint);
    void UpdateServoAngle(JointStatus_t &_joint, float _angleSetPoint);
    void UpdateJointAngle(JointStatus_t &_joint);
    void UpdateJointAngle(JointStatus_t &_joint, float _angleSetPoint);

private:
    RoundLcd *m_pLcd;
    PAJ7620 *m_pPAJ7620;
    UsbRobot *m_pUsbRobot;
    //舵机
    //摄像头
    //手势传感器
    //USB

    uint8_t usbExtraData[32];

};

#endif //ESP_IDF_V4_4_ROBOT_H
