#pragma once

#include <Arduino.h>

class MotoronDrive;

class MotoronChassis
{
public:
    static void drive(
        MotoronDrive& motoron,
        float vx,
        float vy,
        float w
    );

    static void forward(MotoronDrive& motoron, int16_t speed);
    static void backward(MotoronDrive& motoron, int16_t speed);
    static void left(MotoronDrive& motoron, int16_t speed);
    static void right(MotoronDrive& motoron, int16_t speed);
    static void rotate_left(MotoronDrive& motoron, int16_t speed);
    static void rotate_right(MotoronDrive& motoron, int16_t speed);
    static void stop(MotoronDrive& motoron);
};
