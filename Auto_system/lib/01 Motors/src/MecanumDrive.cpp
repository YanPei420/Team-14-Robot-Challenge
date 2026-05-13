#include "MecanumDrive.h"
#include "config.h"
#include "MotorsConfig.h"

MecanumDrive::MecanumDrive(
    MotorDriver* fl,
    MotorDriver* fr,
    MotorDriver* rl,
    MotorDriver* rr
)
{
    FL = fl;
    FR = fr;
    RL = rl;
    RR = rr;
}

void MecanumDrive::move(
    float vx,
    float vy,
    float wz
)
{
    WheelSpeeds ws =
        Kinematics::calculate(
            vx,
            vy,
            wz,
            MAX_MOTOR_SPEED
        );

    FL->setSpeed(ws.frontLeft);
    FR->setSpeed(ws.frontRight);

    RL->setSpeed(ws.rearLeft);
    RR->setSpeed(ws.rearRight);
}

void MecanumDrive::forward(int speed)
{
    move(speed / 800.0f, 0, 0);
}

void MecanumDrive::backward(int speed)
{
    forward(-speed);
}

void MecanumDrive::strafeRight(int speed)
{
    move(0, speed / 800.0f, 0);
}

void MecanumDrive::strafeLeft(int speed)
{
    move(0, -speed / 800.0f, 0);
}

void MecanumDrive::rotateLeft(int speed)
{
    move(0, 0, -speed / 800.0f);
}

void MecanumDrive::rotateRight(int speed)
{
    move(0, 0, speed / 800.0f);
}

void MecanumDrive::stop()
{
    FL->stop();
    FR->stop();
    RL->stop();
    RR->stop();
}