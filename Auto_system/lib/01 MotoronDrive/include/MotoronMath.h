#pragma once

#include <Arduino.h>

struct MotoronWheelSpeeds
{
    int16_t frontLeft;
    int16_t frontRight;
    int16_t rearLeft;
    int16_t rearRight;
};

class MotoronMath
{
public:
    static MotoronWheelSpeeds mecanum(
        float vx,
        float vy,
        float w,
        int16_t maxSpeed
    );

private:
    static float abs_float(float value);
};
