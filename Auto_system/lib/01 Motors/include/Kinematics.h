#ifndef KINEMATICS_H
#define KINEMATICS_H

struct WheelSpeeds
{
    int frontLeft;
    int frontRight;
    int rearLeft;
    int rearRight;
};

class Kinematics
{
public:
    static WheelSpeeds calculate(
        float vx,
        float vy,
        float wz,
        int maxSpeed = 800
    );
};

#endif