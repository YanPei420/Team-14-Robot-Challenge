#ifndef MECANUM_DRIVE_H
#define MECANUM_DRIVE_H

#include "MotorDriver.h"
#include "Kinematics.h"

class MecanumDrive
{
private:
    MotorDriver* FL;
    MotorDriver* FR;
    MotorDriver* RL;
    MotorDriver* RR;

public:
    MecanumDrive(
        MotorDriver* fl,
        MotorDriver* fr,
        MotorDriver* rl,
        MotorDriver* rr
    );

    void move(
        float vx,
        float vy,
        float wz
    );

    void forward(int speed);

    void backward(int speed);

    void strafeRight(int speed);

    void strafeLeft(int speed);

    void rotateLeft(int speed);

    void rotateRight(int speed);

    void stop();
};

#endif