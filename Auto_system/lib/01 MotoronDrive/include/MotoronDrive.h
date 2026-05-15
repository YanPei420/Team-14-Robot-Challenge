#pragma once
#include <Arduino.h>
#define Wire Wire1
#include <Motoron.h>

class MotoronDrive
{
public:
    MotoronDrive(uint8_t Front_Board, uint8_t Rear_Board);

    void begin();
    void stop();

    void set_front_left(int16_t speed);
    void set_front_right(int16_t speed);
    void set_rear_left(int16_t speed);
    void set_rear_right(int16_t speed);

    void set_all(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
    void get_wheel_speeds(int16_t& fl, int16_t& fr, int16_t& rl, int16_t& rr) const;

    void drive(float vx, float vy, float w);
    void set_max_speed(int16_t maxSpeed);
    int16_t get_max_speed() const;

    void forward(int16_t speed);
    void backward(int16_t speed);
    void left(int16_t speed);
    void right(int16_t speed);

    void rotate_left(int16_t speed);
    void rotate_right(int16_t speed);

    void stop_all(); 

private:
    MotoronI2C front_;
    MotoronI2C rear_;

    int16_t maxSpeed_;
    int16_t currentFrontLeft_;
    int16_t currentFrontRight_;
    int16_t currentRearLeft_;
    int16_t currentRearRight_;

    int16_t clamp_speed(int16_t speed) const;
};
