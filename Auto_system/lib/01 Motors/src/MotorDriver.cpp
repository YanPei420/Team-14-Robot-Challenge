#include "MotorDriver.h"
#include "config.h"
#include "MotorsConfig.h"

MotorDriver::MotorDriver(
    MotoronI2C* mc,
    uint8_t motorChannel,
    bool reverse
)
{
    controller = mc;
    channel = motorChannel;
    inverted = reverse;
}

void MotorDriver::setSpeed(int speed)
{
    if (inverted)
    {
        speed = -speed;
    }

    speed = constrain(
        speed,
        -MAX_MOTOR_SPEED,
        MAX_MOTOR_SPEED
    );

    controller->setSpeed(channel, speed);
}

void MotorDriver::stop()
{
    controller->setSpeed(channel, 0);
}