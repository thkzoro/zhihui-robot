#ifndef SERVO_H
#define SERVO_H

#include <cstdint>

class Servo
{
public:
    Servo(){}

    const float DCE_INTEGRAL_LIMIT = 500;

    struct DCE_t
    {
        float kp;
        float kv;
        float ki;
        float kd;
        float setPointPos;
        float setPointVel;
        float integralPos;
        float integralVel;
        float lastError;
        float output;
    };
    DCE_t dce;

    uint16_t adcValAtAngleMin;
    uint16_t adcValAtAngleMax;
    float angle;
    float velocity;
    float mechanicalAngleMin;
    float mechanicalAngleMax;


    void UpdateVelocity();
    void SetEnable(bool _enable);
    void SetTorqueLimit(float _percent);
    float CalcDceOutput(float _inputPos, float _inputVel);
    void SetPwm(int16_t _pwm);

private:
    bool isEnabled;
    float lastAngle;
    float limitAngleMin;
    float limitAngleMax;
    float limitVelocity;
    float limitTorque; // 0~0.1
};

#endif