#include "Planter.h"

Planter::Planter(MotoronDrive& drive)
    : drive_(drive),
      encoder_(PLANTER_ENCODER_A_PIN, PLANTER_ENCODER_B_PIN),
      state_(State::Idle),
      startPosition_(0),
      targetCounts_(PLANTER_COUNTS_PER_HALF_TURN),
      startedMs_(0)
{
}

void Planter::begin()
{
    encoder_.write(0);
    stop();
}

void Planter::startCycle()
{
    startPosition_ = encoder_.read();
    targetCounts_ = PLANTER_COUNTS_PER_HALF_TURN;
    startedMs_ = millis();
    state_ = State::Running;
    writeMotor(PLANTER_MOTOR_SPEED * PLANTER_MOTOR_DIRECTION, true);
}

void Planter::stop()
{
    writeMotor(0, true);
    restoreDriveControl();

    if (state_ == State::Running)
    {
        state_ = State::Idle;
    }
}

bool Planter::update()
{
    if (state_ != State::Running)
    {
        return state_ == State::Complete;
    }

    const long travelled = absoluteLong(encoder_.read() - startPosition_);
    if (travelled >= targetCounts_)
    {
        writeMotor(0, true);
        restoreDriveControl();
        state_ = State::Complete;
        return true;
    }

    if (millis() - startedMs_ >= PLANTER_TIMEOUT_MS)
    {
        writeMotor(0, true);
        restoreDriveControl();
        state_ = State::Timeout;
        return false;
    }

    writeMotor(PLANTER_MOTOR_SPEED * PLANTER_MOTOR_DIRECTION, false);
    return false;
}

bool Planter::isRunning() const
{
    return state_ == State::Running;
}

bool Planter::isComplete() const
{
    return state_ == State::Complete;
}

bool Planter::hasTimedOut() const
{
    return state_ == State::Timeout;
}

Planter::State Planter::state() const
{
    return state_;
}

long Planter::position() const
{
    return const_cast<Encoder&>(encoder_).read();
}

long Planter::targetCounts() const
{
    return targetCounts_;
}

long Planter::absoluteLong(long value) const
{
    return value < 0 ? -value : value;
}

void Planter::writeMotor(int16_t speed, bool immediate)
{
    drive_.raw_rear_motor(PLANTER_MOTOR_CHANNEL, speed, immediate);
}

void Planter::restoreDriveControl()
{
    if (drive_.encoder_speed_control_ready())
    {
        drive_.set_encoder_speed_control_enabled(true);
    }
}
