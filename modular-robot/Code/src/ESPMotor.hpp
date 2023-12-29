
#ifndef _ESPMOTOR_H
#define _ESPMOTOR_H

#include "ESP32Servo.h"

#define FREQ 1000 

class ESPMotor
{
private:
    /* data */
    ESP32PWM pwm1, pwm2;

public:
    ESPMotor(/* args */);
    ~ESPMotor();
    void run(float speed);
    void init(uint8_t pin1, uint8_t pin2);
};

ESPMotor::ESPMotor(/* args */)
{
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
}

ESPMotor::~ESPMotor()
{
}

void ESPMotor::init(uint8_t pin1, uint8_t pin2)
{
    pwm1.attachPin(pin1, FREQ, 10);
    pwm2.attachPin(pin2, FREQ, 10);
}

void ESPMotor::run(float speed)
{

    if (speed < 0.0)
    {
        if (speed < -1.0)
            speed = -1.0;
        pwm1.writeScaled(0.00);
        pwm2.writeScaled(-speed);
    }
    else if (speed >= 0.0)
    {
        if (speed > 1.0)
            speed = 1.0;
        pwm1.writeScaled(speed);
        pwm2.writeScaled(0.00);
    }
}


class _PID
{
private:
    /* data */

public:
    float target_val; 
    float actual_val; 
    float err;        
    float err_last;   
    float Kp, Ki, Kd; 
    float integral;   
    
    _PID();
    float PID_realize(float temp_val);
    void set_p_i_d(float p, float i, float d);
    float get_pid_target(void);
    void set_pid_target(float temp_val);
    void param_init(void);
};

_PID::_PID()
{
    this->target_val = 0.0;
    this->actual_val = 0.0;
    this->err = 0.0;
    this->err_last = 0.0;
    this->integral = 0.0;
    this->Kp = 0.0; // 24
    this->Ki = 0.0;
    this->Kd = 0.0;
}

void _PID::set_pid_target(float temp_val)
{
    this->target_val = temp_val; 
}
float _PID::get_pid_target(void)
{
    return this->target_val; 
}

void _PID::set_p_i_d(float p, float i, float d)
{
    this->Kp = p; 
    this->Ki = i; 
    this->Kd = d; 
}

float _PID::PID_realize(float temp_val)
{
    
    this->err = this->target_val - temp_val;
    
    this->integral += this->err;
    
    this->actual_val = this->Kp * this->err + this->Ki * this->integral + this->Kd * (this->err - this->err_last);
    
    this->err_last = this->err;
    
    return this->actual_val;
}

#endif 
