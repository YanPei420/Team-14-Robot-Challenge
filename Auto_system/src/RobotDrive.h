#pragma once

#include <Arduino.h>

class RobotDrive
{
public:
    virtual ~RobotDrive() = default;

    virtual bool begin() = 0;
    virtual void set_all(
        int16_t frontLeft,
        int16_t frontRight,
        int16_t rearLeft,
        int16_t rearRight
    ) = 0;
    virtual void drive(int16_t vx, int16_t vy, int16_t w) = 0;
    virtual void stop_all() = 0;

    virtual void forward(int16_t speed)
    {
        drive(speed, 0, 0);
    }

    virtual void backward(int16_t speed)
    {
        drive(-speed, 0, 0);
    }

    virtual void left(int16_t speed)
    {
        drive(0, -speed, 0);
    }

    virtual void right(int16_t speed)
    {
        drive(0, speed, 0);
    }

    virtual void rotate_left(int16_t speed)
    {
        drive(0, 0, -speed);
    }

    virtual void rotate_right(int16_t speed)
    {
        drive(0, 0, speed);
    }

    virtual void stop()
    {
        stop_all();
    }
};
