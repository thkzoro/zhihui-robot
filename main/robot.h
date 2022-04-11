#ifndef ESP_IDF_V4_4_ROBOT_H
#define ESP_IDF_V4_4_ROBOT_H
#include "mpu6050.h"
#include "robot_face_screen.h"
#include "paj7620.h"

#define ANY 0

class Robot {
public:
    Robot() {
    	m_pLcd = nullptr; /*m_pPAJ7620 = nullptr; m_pUsbRobot = nullptr;*/
    	m_i2c0_bus_handle = nullptr;
    	m_spi2_bus_handle = nullptr;
        m_mpu6050 = nullptr;
    	m_xQueueFrame_camera = xQueueCreate(2, sizeof(camera_fb_t *));
    	m_xQueueFrame_lcd = xQueueCreate(2, sizeof(camera_fb_t *));
    }
    virtual ~Robot() {
        if (m_pLcd) delete m_pLcd;
        mpu6050_delete(&m_mpu6050);
        /*if (m_pPAJ7620) delete m_pPAJ7620; if (m_pUsbRobot) delete m_pUsbRobot;*/
    }

    esp_err_t robot_init();
    esp_err_t robot_gpio_init(void);
    esp_err_t robot_i2c_bus_init(void);
    esp_err_t robot_spi_bus_init(void);

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
    i2c_bus_handle_t m_i2c0_bus_handle;
    spi_bus_handle_t m_spi2_bus_handle;
    mpu6050_handle_t m_mpu6050;
    QueueHandle_t m_xQueueFrame_camera;
    QueueHandle_t m_xQueueFrame_lcd;
    RobotLCD *m_pLcd;
    PAJ7620 *m_pPAJ7620;

//    UsbRobot *m_pUsbRobot;
    //舵机
    //摄像头
    //手势传感器
    //USB

    uint8_t usbExtraData[32];

};

#endif //ESP_IDF_V4_4_ROBOT_H
