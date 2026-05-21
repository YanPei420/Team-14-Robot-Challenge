#pragma once

#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

#include "MotorConfig.h"

class MotoronDrive
{
private:
    MotoronI2C front_;
    MotoronI2C rear_;

    int16_t maxSpeed_;
    int16_t frontLeft_;
    int16_t frontRight_;
    int16_t rearLeft_;
    int16_t rearRight_;

    int16_t clampSpeed(int16_t speed) const;
    int16_t applyDirection(int16_t speed, int8_t direction) const;
    void setupController(MotoronI2C& controller);
    void setFrontRaw(int16_t motor1, int16_t motor2, bool immediate);
    void setRearRaw(int16_t motor1, int16_t motor2, bool immediate);

public:
    MotoronDrive(
        uint8_t frontAddress = MOTORON_ADDR_FRONT,
        uint8_t rearAddress = MOTORON_ADDR_REAR
    );

    bool begin();

    void set_max_speed(int16_t maxSpeed);
    int16_t get_max_speed() const;

    void set_front_left(int16_t speed);
    void set_front_right(int16_t speed);
    void set_rear_left(int16_t speed);
    void set_rear_right(int16_t speed);
    void set_all(
        int16_t frontLeft,
        int16_t frontRight,
        int16_t rearLeft,
        int16_t rearRight
    );
    void get_wheel_speeds(
        int16_t& frontLeft,
        int16_t& frontRight,
        int16_t& rearLeft,
        int16_t& rearRight
    ) const;

    void drive(int16_t vx, int16_t vy, int16_t w);
    void forward(int16_t speed);
    void backward(int16_t speed);
    void left(int16_t speed);
    void right(int16_t speed);
    void rotate_left(int16_t speed);
    void rotate_right(int16_t speed);

    void stop();
    void stop_all();

    void raw_front(int16_t motor1, int16_t motor2, bool immediate = false);
    void raw_rear(int16_t motor1, int16_t motor2, bool immediate = false);
    void raw_front_motor(
        uint8_t channel,
        int16_t speed,
        bool immediate = false
    );
    void raw_rear_motor(
        uint8_t channel,
        int16_t speed,
        bool immediate = false
    );

    void clear_status_flags();
    void print_status(Stream& output);
};
