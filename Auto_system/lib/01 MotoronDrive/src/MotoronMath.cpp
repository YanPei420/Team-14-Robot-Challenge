#include "MotoronMath.h"

MotoronWheelSpeeds MotoronMath::mecanum(
    float vx,
    float vy,
    float w,
    int16_t maxSpeed
)
{
    float fl = vx - vy - w;
    float fr = vx + vy + w;
    float rl = vx + vy - w;
    float rr = vx - vy + w;

    float maxValue = abs_float(fl);

    if (abs_float(fr) > maxValue)
    {
        maxValue = abs_float(fr);
    }

    if (abs_float(rl) > maxValue)
    {
        maxValue = abs_float(rl);
    }

    if (abs_float(rr) > maxValue)
    {
        maxValue = abs_float(rr);
    }

    if (maxValue > static_cast<float>(maxSpeed))
    {
        const float scale =
            static_cast<float>(maxSpeed)
            /
            maxValue;

        fl *= scale;
        fr *= scale;
        rl *= scale;
        rr *= scale;
    }

    return {
        static_cast<int16_t>(fl),
        static_cast<int16_t>(fr),
        static_cast<int16_t>(rl),
        static_cast<int16_t>(rr)
    };
}

float MotoronMath::abs_float(float value)
{
    return value < 0.0f ? -value : value;
}
