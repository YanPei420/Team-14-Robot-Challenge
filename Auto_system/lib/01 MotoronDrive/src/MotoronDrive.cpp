#include "MotoronDrive.h"
#include "MotorConfig.h"

MotoronDrive::MotoronDrive(uint8_t Front_Board, uint8_t Rear_Board)
    : front_(Front_Board), rear_(Rear_Board)
{
}

void MotoronDrive::begin()
{
    Wire1.begin();

    front_.reinitialize();
    front_.disableCrc();
    front_.clearResetFlag();

    rear_.reinitialize();
    rear_.disableCrc();
    rear_.clearResetFlag();

    stop_all();

    Serial.println("MotoronDrive initialized");
}


int16_t MotoronDrive::clamp_speed(int16_t speed)
{
    if (speed > MOTOR_MAX_SPEED)
    {
        speed = MOTOR_MAX_SPEED;
    }

    if (speed < -MOTOR_MAX_SPEED)
    {
        speed = -MOTOR_MAX_SPEED;
    }

    return speed;
}


void MotoronDrive::set_front_left(int16_t speed)
{
    front_.setSpeed(MOTOR_FL, clamp_speed(speed));
}

void MotoronDrive::set_front_right(int16_t speed)
{
    front_.setSpeed(MOTOR_FR, clamp_speed(speed));
}

void MotoronDrive::set_rear_left(int16_t speed)
{
    rear_.setSpeed(MOTOR_RL, clamp_speed(speed));
}

void MotoronDrive::set_rear_right(int16_t speed)
{
    rear_.setSpeed(MOTOR_RR, clamp_speed(speed));
}


void MotoronDrive::set_all(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    set_front_left(fl);
    set_front_right(fr);
    set_rear_left(rl);
    set_rear_right(rr);
}

void MotoronDrive::forward(int16_t speed)
{
    set_all(speed, speed, speed, speed);
}

void MotoronDrive::backward(int16_t speed)
{
    set_all(-speed, -speed, -speed, -speed);
}

void MotoronDrive::left(int16_t speed)
{
    set_all(-speed, speed, -speed, speed);
}

void MotoronDrive::right(int16_t speed)
{
    set_all(-speed, speed, speed, -speed);
}

void MotoronDrive::rotate_left(int16_t speed)
{
    set_all(-speed, speed, -speed, speed);
}

void MotoronDrive::rotate_right(int16_t speed)
{
    set_all(speed, -speed, speed, -speed);
}

void MotoronDrive::stop_all()
{
    set_all(0, 0, 0, 0);
}