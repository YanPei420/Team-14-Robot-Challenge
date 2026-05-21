#include "MotoronDrive.h"

MotoronDrive::MotoronDrive(uint8_t frontAddress, uint8_t rearAddress)
    : front_(frontAddress),
      rear_(rearAddress),
      maxSpeed_(MOTOR_MAX_SPEED),
      frontLeft_(0),
      frontRight_(0),
      rearLeft_(0),
      rearRight_(0)
{
}

bool MotoronDrive::begin()
{
    Wire1.begin();

    front_.setBus(&Wire1);
    rear_.setBus(&Wire1);

    setupController(front_);
    setupController(rear_);

    stop_all();

    return front_.getLastError() == 0 && rear_.getLastError() == 0;
}

void MotoronDrive::setupController(MotoronI2C& controller)
{
    controller.reinitialize();
    controller.disableCrc();
    controller.clearResetFlag();
    controller.setCommandTimeoutMilliseconds(MOTOR_COMMAND_TIMEOUT_MS);

    controller.setMaxAcceleration(1, MOTOR_MAX_ACCELERATION);
    controller.setMaxDeceleration(1, MOTOR_MAX_DECELERATION);
    controller.setMaxAcceleration(2, MOTOR_MAX_ACCELERATION);
    controller.setMaxDeceleration(2, MOTOR_MAX_DECELERATION);
}

int16_t MotoronDrive::clampSpeed(int16_t speed) const
{
    if (speed > maxSpeed_)
    {
        return maxSpeed_;
    }

    if (speed < -maxSpeed_)
    {
        return -maxSpeed_;
    }

    return speed;
}

int16_t MotoronDrive::applyDirection(int16_t speed, int8_t direction) const
{
    return clampSpeed(speed) * direction;
}

void MotoronDrive::set_max_speed(int16_t maxSpeed)
{
    if (maxSpeed < 0)
    {
        maxSpeed = -maxSpeed;
    }

    maxSpeed_ = maxSpeed;
}

int16_t MotoronDrive::get_max_speed() const
{
    return maxSpeed_;
}

void MotoronDrive::set_front_left(int16_t speed)
{
    frontLeft_ = clampSpeed(speed);
    front_.setSpeed(
        MOTORON_CHANNEL_FRONT_LEFT,
        applyDirection(frontLeft_, MOTOR_FRONT_LEFT_DIRECTION)
    );
}

void MotoronDrive::set_front_right(int16_t speed)
{
    frontRight_ = clampSpeed(speed);
    front_.setSpeed(
        MOTORON_CHANNEL_FRONT_RIGHT,
        applyDirection(frontRight_, MOTOR_FRONT_RIGHT_DIRECTION)
    );
}

void MotoronDrive::set_rear_left(int16_t speed)
{
    rearLeft_ = clampSpeed(speed);
    rear_.setSpeed(
        MOTORON_CHANNEL_REAR_LEFT,
        applyDirection(rearLeft_, MOTOR_REAR_LEFT_DIRECTION)
    );
}

void MotoronDrive::set_rear_right(int16_t speed)
{
    rearRight_ = clampSpeed(speed);
    rear_.setSpeed(
        MOTORON_CHANNEL_REAR_RIGHT,
        applyDirection(rearRight_, MOTOR_REAR_RIGHT_DIRECTION)
    );
}

void MotoronDrive::set_all(
    int16_t frontLeft,
    int16_t frontRight,
    int16_t rearLeft,
    int16_t rearRight
)
{
    frontLeft_ = clampSpeed(frontLeft);
    frontRight_ = clampSpeed(frontRight);
    rearLeft_ = clampSpeed(rearLeft);
    rearRight_ = clampSpeed(rearRight);

    front_.setSpeed(
        MOTORON_CHANNEL_FRONT_LEFT,
        applyDirection(frontLeft_, MOTOR_FRONT_LEFT_DIRECTION)
    );
    front_.setSpeed(
        MOTORON_CHANNEL_FRONT_RIGHT,
        applyDirection(frontRight_, MOTOR_FRONT_RIGHT_DIRECTION)
    );
    rear_.setSpeed(
        MOTORON_CHANNEL_REAR_LEFT,
        applyDirection(rearLeft_, MOTOR_REAR_LEFT_DIRECTION)
    );
    rear_.setSpeed(
        MOTORON_CHANNEL_REAR_RIGHT,
        applyDirection(rearRight_, MOTOR_REAR_RIGHT_DIRECTION)
    );
}

void MotoronDrive::get_wheel_speeds(
    int16_t& frontLeft,
    int16_t& frontRight,
    int16_t& rearLeft,
    int16_t& rearRight
) const
{
    frontLeft = frontLeft_;
    frontRight = frontRight_;
    rearLeft = rearLeft_;
    rearRight = rearRight_;
}

void MotoronDrive::drive(int16_t vx, int16_t vy, int16_t w)
{
    int32_t frontLeft = static_cast<int32_t>(vx) - vy - w;
    int32_t frontRight = static_cast<int32_t>(vx) + vy + w;
    int32_t rearLeft = static_cast<int32_t>(vx) + vy - w;
    int32_t rearRight = static_cast<int32_t>(vx) - vy + w;

    int32_t largest = abs(frontLeft);

    if (abs(frontRight) > largest)
    {
        largest = abs(frontRight);
    }

    if (abs(rearLeft) > largest)
    {
        largest = abs(rearLeft);
    }

    if (abs(rearRight) > largest)
    {
        largest = abs(rearRight);
    }

    if (largest > maxSpeed_ && largest != 0)
    {
        frontLeft = frontLeft * maxSpeed_ / largest;
        frontRight = frontRight * maxSpeed_ / largest;
        rearLeft = rearLeft * maxSpeed_ / largest;
        rearRight = rearRight * maxSpeed_ / largest;
    }

    set_all(
        static_cast<int16_t>(frontLeft),
        static_cast<int16_t>(frontRight),
        static_cast<int16_t>(rearLeft),
        static_cast<int16_t>(rearRight)
    );
}

void MotoronDrive::forward(int16_t speed)
{
    drive(speed, 0, 0);
}

void MotoronDrive::backward(int16_t speed)
{
    drive(-speed, 0, 0);
}

void MotoronDrive::left(int16_t speed)
{
    drive(0, -speed, 0);
}

void MotoronDrive::right(int16_t speed)
{
    drive(0, speed, 0);
}

void MotoronDrive::rotate_left(int16_t speed)
{
    drive(0, 0, -speed);
}

void MotoronDrive::rotate_right(int16_t speed)
{
    drive(0, 0, speed);
}

void MotoronDrive::stop()
{
    stop_all();
}

void MotoronDrive::stop_all()
{
    frontLeft_ = 0;
    frontRight_ = 0;
    rearLeft_ = 0;
    rearRight_ = 0;

    front_.setSpeedNow(MOTORON_CHANNEL_FRONT_LEFT, 0);
    front_.setSpeedNow(MOTORON_CHANNEL_FRONT_RIGHT, 0);
    rear_.setSpeedNow(MOTORON_CHANNEL_REAR_LEFT, 0);
    rear_.setSpeedNow(MOTORON_CHANNEL_REAR_RIGHT, 0);
}

void MotoronDrive::setFrontRaw(int16_t motor1, int16_t motor2, bool immediate)
{
    motor1 = clampSpeed(motor1);
    motor2 = clampSpeed(motor2);

    if (immediate)
    {
        front_.setSpeedNow(1, motor1);
        front_.setSpeedNow(2, motor2);
    }
    else
    {
        front_.setSpeed(1, motor1);
        front_.setSpeed(2, motor2);
    }
}

void MotoronDrive::setRearRaw(int16_t motor1, int16_t motor2, bool immediate)
{
    motor1 = clampSpeed(motor1);
    motor2 = clampSpeed(motor2);

    if (immediate)
    {
        rear_.setSpeedNow(1, motor1);
        rear_.setSpeedNow(2, motor2);
    }
    else
    {
        rear_.setSpeed(1, motor1);
        rear_.setSpeed(2, motor2);
    }
}

void MotoronDrive::raw_front(
    int16_t motor1,
    int16_t motor2,
    bool immediate
)
{
    frontLeft_ = clampSpeed(motor1);
    frontRight_ = clampSpeed(motor2);
    setFrontRaw(motor1, motor2, immediate);
}

void MotoronDrive::raw_rear(int16_t motor1, int16_t motor2, bool immediate)
{
    rearLeft_ = clampSpeed(motor1);
    rearRight_ = clampSpeed(motor2);
    setRearRaw(motor1, motor2, immediate);
}

void MotoronDrive::raw_front_motor(
    uint8_t channel,
    int16_t speed,
    bool immediate
)
{
    speed = clampSpeed(speed);

    if (channel == 1)
    {
        frontLeft_ = speed;
    }
    else if (channel == 2)
    {
        frontRight_ = speed;
    }

    if (immediate)
    {
        front_.setSpeedNow(channel, speed);
    }
    else
    {
        front_.setSpeed(channel, speed);
    }
}

void MotoronDrive::raw_rear_motor(
    uint8_t channel,
    int16_t speed,
    bool immediate
)
{
    speed = clampSpeed(speed);

    if (channel == 1)
    {
        rearLeft_ = speed;
    }
    else if (channel == 2)
    {
        rearRight_ = speed;
    }

    if (immediate)
    {
        rear_.setSpeedNow(channel, speed);
    }
    else
    {
        rear_.setSpeed(channel, speed);
    }
}

void MotoronDrive::clear_status_flags()
{
    front_.clearResetFlag();
    rear_.clearResetFlag();
    front_.clearLatchedStatusFlags(0xFFFF);
    rear_.clearLatchedStatusFlags(0xFFFF);
    front_.clearMotorFaultUnconditional();
    rear_.clearMotorFaultUnconditional();
}

void MotoronDrive::print_status(Stream& output)
{
    output.print("Front Motoron 0x");
    output.print(front_.getAddress(), HEX);
    output.print(" status=0x");
    output.print(front_.getStatusFlags(), HEX);
    output.print(" lastError=");
    output.println(front_.getLastError());

    output.print("Rear Motoron 0x");
    output.print(rear_.getAddress(), HEX);
    output.print(" status=0x");
    output.print(rear_.getStatusFlags(), HEX);
    output.print(" lastError=");
    output.println(rear_.getLastError());
}
