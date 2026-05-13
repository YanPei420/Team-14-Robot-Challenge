#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#define Wire Wire1

#include <Arduino.h>
#include <Motoron.h>

class MotorDriver
{
private:
    MotoronI2C* controller;
    uint8_t channel;
    bool inverted;

public:
    MotorDriver(
        MotoronI2C* mc,
        uint8_t motorChannel,
        bool reverse = false
    );

    void setSpeed(int speed);

    void stop();
};

#endif