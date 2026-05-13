#include "Kinematics.h"

#include <Arduino.h>

WheelSpeeds Kinematics::calculate(
    float vx,
    float vy,
    float wz,
    int maxSpeed
)
{
    WheelSpeeds ws;

    float fl = vx - vy - wz;
    float fr = vx + vy + wz;

    float rl = vx + vy - wz;
    float rr = vx - vy + wz;

    float maxVal = max(
        max(abs(fl), abs(fr)),
        max(abs(rl), abs(rr))
    );

    if (maxVal > 1.0f)
    {
        fl /= maxVal;
        fr /= maxVal;
        rl /= maxVal;
        rr /= maxVal;
    }

    ws.frontLeft  = fl * maxSpeed;
    ws.frontRight = fr * maxSpeed;

    ws.rearLeft   = rl * maxSpeed;
    ws.rearRight  = rr * maxSpeed;

    return ws;
}