#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Motoron.h>

class MotorControl {
public:
    MotorControl();
    void begin();
    void setSpeed(int speed);
    void stop();
private:
    MotoronI2C _motors;
};

#endif
