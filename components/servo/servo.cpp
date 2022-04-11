#include "servo.h"

#define PWM1_OUTPUT_IO          (5) // Define the output GPIO
#define PWM2_OUTPUT_IO          (6) // Define the output GPIO


void Servo::servoPwmTimerInit()
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
       .speed_mode       = LEDC_LOW_SPEED_MODE,
       .duty_resolution  = LEDC_TIMER_10_BIT,
       .timer_num        = LEDC_TIMER_0,
       .freq_hz          = 1000,  // Set output frequency at 1 kHz
       .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

void Servo::servoPwmInit()
{
    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t channel_1 = {
       .gpio_num       = PWM1_OUTPUT_IO,
       .speed_mode     = LEDC_LOW_SPEED_MODE,
       .channel        = m_pwm1,
       .intr_type      = LEDC_INTR_DISABLE,
       .timer_sel      = LEDC_TIMER_0,
       .duty           = 0, // Set duty to 0% 0~1023
       .hpoint         = 0
    };
       
    ESP_ERROR_CHECK(ledc_channel_config(&channel_1));

    ledc_channel_config_t channel_2 = {
        .gpio_num       = PWM2_OUTPUT_IO,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = m_pwm2,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_2));
}

void Servo::servoAdcInit() 
{

}

float Servo::CalcDceOutput(float _inputPos, float _inputVel)
{
    float errorPos = _inputPos - dce.setPointPos;
    float errorVel = _inputVel - dce.setPointVel;
    float deltaPos = errorPos - dce.lastError;
    dce.lastError = errorPos;
    dce.integralPos += errorPos;
    if (dce.integralPos > DCE_INTEGRAL_LIMIT) dce.integralPos = DCE_INTEGRAL_LIMIT;
    else if (dce.integralPos < -DCE_INTEGRAL_LIMIT) dce.integralPos = -DCE_INTEGRAL_LIMIT;
    dce.integralVel += errorVel;
    if (dce.integralVel > DCE_INTEGRAL_LIMIT) dce.integralVel = DCE_INTEGRAL_LIMIT;
    else if (dce.integralVel < -DCE_INTEGRAL_LIMIT) dce.integralVel = -DCE_INTEGRAL_LIMIT;

    dce.output = dce.kp * errorPos +
                 dce.ki * dce.integralPos + dce.kv * dce.integralVel +
                 dce.kd * deltaPos;

    if (dce.output > limitTorque) dce.output = limitTorque;
    else if (dce.output < -limitTorque) dce.output = -limitTorque;

    return dce.output;
}


void Servo::SetPwm(int16_t _pwm)
{
    if (!isEnabled)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, m_pwm1, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, m_pwm2, 0);
        return;
    }
    
    if (_pwm >= 0)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, m_pwm1, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, m_pwm2, _pwm > 1000 ? 1000 : _pwm);
    } else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, m_pwm2, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, m_pwm1, _pwm > 1000 ? 1000 : _pwm);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, m_pwm1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m_pwm2);
}


void Servo::SetTorqueLimit(float _percent)
{
    if (_percent > 1)_percent = 1;
    else if (_percent < 0)_percent = 0;

    limitTorque = _percent * 1000;
}


void Servo::UpdateVelocity()
{
    velocity = angle - lastAngle;
    lastAngle = angle;
}


void Servo::SetEnable(bool _enable)
{
    isEnabled = _enable;
}

