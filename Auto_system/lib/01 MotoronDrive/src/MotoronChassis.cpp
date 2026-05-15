#include "MotoronChassis.h"

#include "MotoronDrive.h"
#include "MotoronMath.h"

void MotoronChassis::drive(
    MotoronDrive& motoron,
    float vx,
    float vy,
    float w
)
{
    const MotoronWheelSpeeds speeds =
        MotoronMath::mecanum(
            vx,
            vy,
            w,
            motoron.get_max_speed()
        );

    motoron.set_all(
        speeds.frontLeft,
        speeds.frontRight,
        speeds.rearLeft,
        speeds.rearRight
    );
}

void MotoronChassis::forward(MotoronDrive& motoron, int16_t speed)
{
    drive(
        motoron,
        speed,
        0.0f,
        0.0f
    );
}

void MotoronChassis::backward(MotoronDrive& motoron, int16_t speed)
{
    drive(
        motoron,
        -speed,
        0.0f,
        0.0f
    );
}

void MotoronChassis::left(MotoronDrive& motoron, int16_t speed)
{
    drive(
        motoron,
        0.0f,
        -speed,
        0.0f
    );
}

void MotoronChassis::right(MotoronDrive& motoron, int16_t speed)
{
    drive(
        motoron,
        0.0f,
        speed,
        0.0f
    );
}

void MotoronChassis::rotate_left(MotoronDrive& motoron, int16_t speed)
{
    drive(
        motoron,
        0.0f,
        0.0f,
        speed
    );
}

void MotoronChassis::rotate_right(MotoronDrive& motoron, int16_t speed)
{
    drive(
        motoron,
        0.0f,
        0.0f,
        -speed
    );
}

void MotoronChassis::stop(MotoronDrive& motoron)
{
    motoron.stop_all();
}
