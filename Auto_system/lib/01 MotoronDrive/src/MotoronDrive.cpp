#include "MotoronDrive.h"

#include "MotoronChassis.h"
#include "MotorConfig.h"

MotoronDrive::MotoronDrive(uint8_t Front_Board, uint8_t Rear_Board)
    : front_(Front_Board),
      rear_(Rear_Board)
{
    maxSpeed_ = MOTOR_MAX_SPEED;
    currentFrontLeft_ = 0;
    currentFrontRight_ = 0;
    currentRearLeft_ = 0;
    currentRearRight_ = 0;
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

void MotoronDrive::stop()
{
    MotoronChassis::stop(*this);
}

int16_t MotoronDrive::clamp_speed(int16_t speed) const
{
    if (speed > maxSpeed_)
    {
        speed = maxSpeed_;
    }

    if (speed < -maxSpeed_)
    {
        speed = -maxSpeed_;
    }

    return speed;
}


void MotoronDrive::set_front_left(int16_t speed)
{
    currentFrontLeft_ = clamp_speed(speed);
    front_.setSpeed(MOTOR_FL, currentFrontLeft_);
}

void MotoronDrive::set_front_right(int16_t speed)
{
    currentFrontRight_ = clamp_speed(speed);
    front_.setSpeed(MOTOR_FR, currentFrontRight_);
}

void MotoronDrive::set_rear_left(int16_t speed)
{
    currentRearLeft_ = clamp_speed(speed);
    rear_.setSpeed(MOTOR_RL, currentRearLeft_);
}

void MotoronDrive::set_rear_right(int16_t speed)
{
    currentRearRight_ = clamp_speed(speed);
    rear_.setSpeed(MOTOR_RR, currentRearRight_);
}


void MotoronDrive::set_all(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    set_front_left(fl);
    set_front_right(fr);
    set_rear_left(rl);
    set_rear_right(rr);
}

void MotoronDrive::get_wheel_speeds(int16_t& fl, int16_t& fr, int16_t& rl, int16_t& rr) const
{
    fl = currentFrontLeft_;
    fr = currentFrontRight_;
    rl = currentRearLeft_;
    rr = currentRearRight_;
}

void MotoronDrive::drive(float vx, float vy, float w)
{
    MotoronChassis::drive(
        *this,
        vx,
        vy,
        w
    );
}

void MotoronDrive::set_max_speed(int16_t maxSpeed)
{
    if (maxSpeed < 0)
    {
        maxSpeed = -maxSpeed;
    }

    if (maxSpeed > MOTOR_MAX_SPEED)
    {
        maxSpeed = MOTOR_MAX_SPEED;
    }

    maxSpeed_ = maxSpeed;
}

int16_t MotoronDrive::get_max_speed() const
{
    return maxSpeed_;
}

void MotoronDrive::forward(int16_t speed)
{
    MotoronChassis::forward(*this, speed);
}

void MotoronDrive::backward(int16_t speed)
{
    MotoronChassis::backward(*this, speed);
}

void MotoronDrive::left(int16_t speed)
{
    MotoronChassis::left(*this, speed);
}

void MotoronDrive::right(int16_t speed)
{
    MotoronChassis::right(*this, speed);
}

void MotoronDrive::rotate_left(int16_t speed)
{
    MotoronChassis::rotate_left(*this, speed);
}

void MotoronDrive::rotate_right(int16_t speed)
{
    MotoronChassis::rotate_right(*this, speed);
}

void MotoronDrive::stop_all()
{
    set_all(0, 0, 0, 0);
}
