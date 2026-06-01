#include "MotoronDrive.h"

MotoronDrive::MotoronDrive(uint8_t frontAddress, uint8_t rearAddress)
    : front_(frontAddress),
      rear_(rearAddress),
      encoders_(),
      maxSpeed_(MOTOR_MAX_SPEED),
      targetSpeed_{0, 0, 0, 0},
      outputSpeed_{0, 0, 0, 0},
      speedControlConfig_(),
      encoderSpeedControlReady_(false),
      encoderSpeedControlEnabled_(false),
      lastSpeedControlMs_(0),
      measuredRPM_{0.0f, 0.0f, 0.0f, 0.0f},
      targetRPM_{0.0f, 0.0f, 0.0f, 0.0f},
      speedError_{0.0f, 0.0f, 0.0f, 0.0f},
      lastSpeedError_{0.0f, 0.0f, 0.0f, 0.0f},
      speedIntegral_{0.0f, 0.0f, 0.0f, 0.0f},
      speedCorrection_{0.0f, 0.0f, 0.0f, 0.0f}
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

    if (MOTOR_SPEED_CONTROL_AUTO_BEGIN)
    {
        begin_encoder_speed_control();
    }

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
    controller.setMaxAcceleration(3, MOTOR_MAX_ACCELERATION);
    controller.setMaxDeceleration(3, MOTOR_MAX_DECELERATION);
}

void MotoronDrive::resetEncoderSpeedControlState()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        resetWheelSpeedControlState(i);
    }

    lastSpeedControlMs_ = millis();

    if (encoderSpeedControlReady_)
    {
        encoders_.reset_counts();
    }
}

void MotoronDrive::resetWheelSpeedControlState(uint8_t wheel)
{
    if (wheel >= WHEEL_COUNT)
    {
        return;
    }

    measuredRPM_[wheel] = 0.0f;
    targetRPM_[wheel] = 0.0f;
    speedError_[wheel] = 0.0f;
    lastSpeedError_[wheel] = 0.0f;
    speedIntegral_[wheel] = 0.0f;
    speedCorrection_[wheel] = 0.0f;
}

void MotoronDrive::setWheelTarget(uint8_t wheel, int16_t speed)
{
    if (wheel >= WHEEL_COUNT)
    {
        return;
    }

    speed = clampSpeed(speed);

    const int16_t oldSpeed = targetSpeed_[wheel];
    const bool directionChanged =
        (oldSpeed > 0 && speed < 0)
        || (oldSpeed < 0 && speed > 0);
    const bool startingOrStopping = oldSpeed == 0 || speed == 0;

    targetSpeed_[wheel] = speed;

    if (directionChanged || startingOrStopping)
    {
        resetWheelSpeedControlState(wheel);
    }
}

void MotoronDrive::setTargetSpeeds(
    int16_t frontLeft,
    int16_t frontRight,
    int16_t rearLeft,
    int16_t rearRight
)
{
    setWheelTarget(FRONT_LEFT, frontLeft);
    setWheelTarget(FRONT_RIGHT, frontRight);
    setWheelTarget(REAR_LEFT, rearLeft);
    setWheelTarget(REAR_RIGHT, rearRight);
}

void MotoronDrive::writeWheelOutputs(
    int16_t frontLeft,
    int16_t frontRight,
    int16_t rearLeft,
    int16_t rearRight,
    bool immediate
)
{
    outputSpeed_[FRONT_LEFT] = clampSpeed(frontLeft);
    outputSpeed_[FRONT_RIGHT] = clampSpeed(frontRight);
    outputSpeed_[REAR_LEFT] = clampSpeed(rearLeft);
    outputSpeed_[REAR_RIGHT] = clampSpeed(rearRight);

    if (immediate)
    {
        front_.setSpeedNow(
            MOTORON_CHANNEL_FRONT_LEFT,
            applyDirection(outputSpeed_[FRONT_LEFT], MOTOR_FRONT_LEFT_DIRECTION)
        );
        front_.setSpeedNow(
            MOTORON_CHANNEL_FRONT_RIGHT,
            applyDirection(outputSpeed_[FRONT_RIGHT], MOTOR_FRONT_RIGHT_DIRECTION)
        );
        rear_.setSpeedNow(
            MOTORON_CHANNEL_REAR_LEFT,
            applyDirection(outputSpeed_[REAR_LEFT], MOTOR_REAR_LEFT_DIRECTION)
        );
        rear_.setSpeedNow(
            MOTORON_CHANNEL_REAR_RIGHT,
            applyDirection(outputSpeed_[REAR_RIGHT], MOTOR_REAR_RIGHT_DIRECTION)
        );
        return;
    }

    front_.setSpeed(
        MOTORON_CHANNEL_FRONT_LEFT,
        applyDirection(outputSpeed_[FRONT_LEFT], MOTOR_FRONT_LEFT_DIRECTION)
    );
    front_.setSpeed(
        MOTORON_CHANNEL_FRONT_RIGHT,
        applyDirection(outputSpeed_[FRONT_RIGHT], MOTOR_FRONT_RIGHT_DIRECTION)
    );
    rear_.setSpeed(
        MOTORON_CHANNEL_REAR_LEFT,
        applyDirection(outputSpeed_[REAR_LEFT], MOTOR_REAR_LEFT_DIRECTION)
    );
    rear_.setSpeed(
        MOTORON_CHANNEL_REAR_RIGHT,
        applyDirection(outputSpeed_[REAR_RIGHT], MOTOR_REAR_RIGHT_DIRECTION)
    );
}

void MotoronDrive::writeTargetsWithCorrections(bool immediate)
{
    int16_t output[WHEEL_COUNT];

    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
    {
        if (targetSpeed_[i] == 0)
        {
            output[i] = 0;
            continue;
        }

        const float corrected = targetSpeed_[i] + speedCorrection_[i];
        output[i] = clampSpeed(static_cast<int16_t>(corrected));

        if (targetSpeed_[i] > 0 && output[i] < 0)
        {
            output[i] = 0;
        }
        else if (targetSpeed_[i] < 0 && output[i] > 0)
        {
            output[i] = 0;
        }
    }

    writeWheelOutputs(
        output[FRONT_LEFT],
        output[FRONT_RIGHT],
        output[REAR_LEFT],
        output[REAR_RIGHT],
        immediate
    );
}

void MotoronDrive::applyTargetSpeeds(bool immediate)
{
    if (encoderSpeedControlEnabled_ && encoderSpeedControlReady_)
    {
        if (update_encoder_speed_control())
        {
            return;
        }
    }

    writeTargetsWithCorrections(immediate);
}

float MotoronDrive::absFloat(float value) const
{
    return value < 0.0f ? -value : value;
}

float MotoronDrive::targetSpeedToRPM(int16_t speed) const
{
    if (maxSpeed_ <= 0 || speedControlConfig_.maxWheelRPM <= 0.0f)
    {
        return 0.0f;
    }

    return static_cast<float>(speed)
        * speedControlConfig_.maxWheelRPM
        / static_cast<float>(maxSpeed_);
}

int16_t MotoronDrive::computeControlledOutput(
    uint8_t wheel,
    int16_t targetSpeed,
    float measuredRPM,
    float dt
)
{
    targetRPM_[wheel] = targetSpeedToRPM(targetSpeed);

    if (targetSpeed == 0)
    {
        speedError_[wheel] = 0.0f;
        lastSpeedError_[wheel] = 0.0f;
        speedIntegral_[wheel] = 0.0f;
        speedCorrection_[wheel] = 0.0f;
        return 0;
    }

    float error = targetRPM_[wheel] - measuredRPM;

    if (absFloat(error) < speedControlConfig_.rpmDeadband)
    {
        error = 0.0f;
    }

    speedError_[wheel] = error;
    speedIntegral_[wheel] += error * dt;

    if (speedIntegral_[wheel] > speedControlConfig_.integralLimit)
    {
        speedIntegral_[wheel] = speedControlConfig_.integralLimit;
    }
    else if (speedIntegral_[wheel] < -speedControlConfig_.integralLimit)
    {
        speedIntegral_[wheel] = -speedControlConfig_.integralLimit;
    }

    const float derivative = dt > 0.0f
        ? (error - lastSpeedError_[wheel]) / dt
        : 0.0f;

    float correction =
        speedControlConfig_.kp * error
        + speedControlConfig_.ki * speedIntegral_[wheel]
        + speedControlConfig_.kd * derivative;

    const int16_t maxCorrection =
        speedControlConfig_.maxCorrection < 0
            ? -speedControlConfig_.maxCorrection
            : speedControlConfig_.maxCorrection;

    if (correction > maxCorrection)
    {
        correction = maxCorrection;
    }
    else if (correction < -maxCorrection)
    {
        correction = -maxCorrection;
    }

    speedCorrection_[wheel] = correction;
    lastSpeedError_[wheel] = error;

    int16_t output = clampSpeed(static_cast<int16_t>(targetSpeed + correction));

    if (targetSpeed > 0 && output < 0)
    {
        output = 0;
    }
    else if (targetSpeed < 0 && output > 0)
    {
        output = 0;
    }

    const int16_t minOutput =
        speedControlConfig_.minOutput < 0
            ? -speedControlConfig_.minOutput
            : speedControlConfig_.minOutput;

    if (targetSpeed > 0 && output > 0 && output < minOutput)
    {
        output = minOutput;
    }
    else if (targetSpeed < 0 && output < 0 && output > -minOutput)
    {
        output = -minOutput;
    }

    return clampSpeed(output);
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

bool MotoronDrive::begin_encoder_speed_control(
    const MotoronSpeedControlConfig& config
)
{
    speedControlConfig_ = config;

    if (!encoderSpeedControlReady_)
    {
        encoderSpeedControlReady_ = encoders_.begin();
    }

    resetEncoderSpeedControlState();
    encoderSpeedControlEnabled_ = encoderSpeedControlReady_;
    return encoderSpeedControlReady_;
}

bool MotoronDrive::set_encoder_speed_control_enabled(bool enabled)
{
    if (enabled && !encoderSpeedControlReady_)
    {
        return false;
    }

    encoderSpeedControlEnabled_ = enabled;
    resetEncoderSpeedControlState();

    if (!enabled)
    {
        writeWheelOutputs(
            targetSpeed_[FRONT_LEFT],
            targetSpeed_[FRONT_RIGHT],
            targetSpeed_[REAR_LEFT],
            targetSpeed_[REAR_RIGHT]
        );
    }

    return true;
}

bool MotoronDrive::encoder_speed_control_enabled() const
{
    return encoderSpeedControlEnabled_;
}

bool MotoronDrive::encoder_speed_control_ready() const
{
    return encoderSpeedControlReady_;
}

bool MotoronDrive::update_encoder_speed_control()
{
    if (!encoderSpeedControlEnabled_ || !encoderSpeedControlReady_)
    {
        return false;
    }

    encoders_.poll();

    const uint32_t now = millis();
    if (now - lastSpeedControlMs_ < speedControlConfig_.intervalMs)
    {
        return false;
    }

    const float dt = (now - lastSpeedControlMs_) / 1000.0f;
    lastSpeedControlMs_ = now;

    encoders_.sample_rpm(
        measuredRPM_[FRONT_LEFT],
        measuredRPM_[FRONT_RIGHT],
        measuredRPM_[REAR_LEFT],
        measuredRPM_[REAR_RIGHT]
    );

    const int16_t frontLeft = computeControlledOutput(
        FRONT_LEFT,
        targetSpeed_[FRONT_LEFT],
        measuredRPM_[FRONT_LEFT],
        dt
    );
    const int16_t frontRight = computeControlledOutput(
        FRONT_RIGHT,
        targetSpeed_[FRONT_RIGHT],
        measuredRPM_[FRONT_RIGHT],
        dt
    );
    const int16_t rearLeft = computeControlledOutput(
        REAR_LEFT,
        targetSpeed_[REAR_LEFT],
        measuredRPM_[REAR_LEFT],
        dt
    );
    const int16_t rearRight = computeControlledOutput(
        REAR_RIGHT,
        targetSpeed_[REAR_RIGHT],
        measuredRPM_[REAR_RIGHT],
        dt
    );

    writeWheelOutputs(frontLeft, frontRight, rearLeft, rearRight);
    return true;
}

bool MotoronDrive::update()
{
    return update_encoder_speed_control();
}

void MotoronDrive::reset_encoder_speed_control()
{
    resetEncoderSpeedControlState();
}

void MotoronDrive::set_encoder_speed_control_config(
    const MotoronSpeedControlConfig& config
)
{
    speedControlConfig_ = config;
    resetEncoderSpeedControlState();
}

MotoronSpeedControlConfig MotoronDrive::get_encoder_speed_control_config() const
{
    return speedControlConfig_;
}

void MotoronDrive::set_front_left(int16_t speed)
{
    setWheelTarget(FRONT_LEFT, speed);
    applyTargetSpeeds();
}

void MotoronDrive::set_front_right(int16_t speed)
{
    setWheelTarget(FRONT_RIGHT, speed);
    applyTargetSpeeds();
}

void MotoronDrive::set_rear_left(int16_t speed)
{
    setWheelTarget(REAR_LEFT, speed);
    applyTargetSpeeds();
}

void MotoronDrive::set_rear_right(int16_t speed)
{
    setWheelTarget(REAR_RIGHT, speed);
    applyTargetSpeeds();
}

void MotoronDrive::set_all(
    int16_t frontLeft,
    int16_t frontRight,
    int16_t rearLeft,
    int16_t rearRight
)
{
    setTargetSpeeds(frontLeft, frontRight, rearLeft, rearRight);
    applyTargetSpeeds();
}

void MotoronDrive::get_wheel_speeds(
    int16_t& frontLeft,
    int16_t& frontRight,
    int16_t& rearLeft,
    int16_t& rearRight
) const
{
    frontLeft = targetSpeed_[FRONT_LEFT];
    frontRight = targetSpeed_[FRONT_RIGHT];
    rearLeft = targetSpeed_[REAR_LEFT];
    rearRight = targetSpeed_[REAR_RIGHT];
}

void MotoronDrive::get_applied_wheel_speeds(
    int16_t& frontLeft,
    int16_t& frontRight,
    int16_t& rearLeft,
    int16_t& rearRight
) const
{
    frontLeft = outputSpeed_[FRONT_LEFT];
    frontRight = outputSpeed_[FRONT_RIGHT];
    rearLeft = outputSpeed_[REAR_LEFT];
    rearRight = outputSpeed_[REAR_RIGHT];
}

void MotoronDrive::get_encoder_counts(
    int32_t& frontLeft,
    int32_t& frontRight,
    int32_t& rearLeft,
    int32_t& rearRight
) const
{
    encoders_.get_counts(frontLeft, frontRight, rearLeft, rearRight);
}

void MotoronDrive::get_encoder_rpm(
    float& frontLeft,
    float& frontRight,
    float& rearLeft,
    float& rearRight
) const
{
    frontLeft = measuredRPM_[FRONT_LEFT];
    frontRight = measuredRPM_[FRONT_RIGHT];
    rearLeft = measuredRPM_[REAR_LEFT];
    rearRight = measuredRPM_[REAR_RIGHT];
}

void MotoronDrive::get_target_rpm(
    float& frontLeft,
    float& frontRight,
    float& rearLeft,
    float& rearRight
) const
{
    frontLeft = targetRPM_[FRONT_LEFT];
    frontRight = targetRPM_[FRONT_RIGHT];
    rearLeft = targetRPM_[REAR_LEFT];
    rearRight = targetRPM_[REAR_RIGHT];
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
    setTargetSpeeds(0, 0, 0, 0);
    resetEncoderSpeedControlState();
    writeWheelOutputs(0, 0, 0, 0, true);
    rear_.setSpeedNow(3, 0);
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
    encoderSpeedControlEnabled_ = false;
    resetEncoderSpeedControlState();

    targetSpeed_[FRONT_LEFT] = clampSpeed(motor1);
    targetSpeed_[FRONT_RIGHT] = clampSpeed(motor2);
    outputSpeed_[FRONT_LEFT] = targetSpeed_[FRONT_LEFT];
    outputSpeed_[FRONT_RIGHT] = targetSpeed_[FRONT_RIGHT];
    setFrontRaw(motor1, motor2, immediate);
}

void MotoronDrive::raw_rear(int16_t motor1, int16_t motor2, bool immediate)
{
    encoderSpeedControlEnabled_ = false;
    resetEncoderSpeedControlState();

    targetSpeed_[REAR_LEFT] = clampSpeed(motor1);
    targetSpeed_[REAR_RIGHT] = clampSpeed(motor2);
    outputSpeed_[REAR_LEFT] = targetSpeed_[REAR_LEFT];
    outputSpeed_[REAR_RIGHT] = targetSpeed_[REAR_RIGHT];
    setRearRaw(motor1, motor2, immediate);
}

void MotoronDrive::raw_front_motor(
    uint8_t channel,
    int16_t speed,
    bool immediate
)
{
    encoderSpeedControlEnabled_ = false;
    resetEncoderSpeedControlState();

    speed = clampSpeed(speed);

    if (channel == 1)
    {
        targetSpeed_[FRONT_LEFT] = speed;
        outputSpeed_[FRONT_LEFT] = speed;
    }
    else if (channel == 2)
    {
        targetSpeed_[FRONT_RIGHT] = speed;
        outputSpeed_[FRONT_RIGHT] = speed;
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
    encoderSpeedControlEnabled_ = false;
    resetEncoderSpeedControlState();

    speed = clampSpeed(speed);

    if (channel == 1)
    {
        targetSpeed_[REAR_LEFT] = speed;
        outputSpeed_[REAR_LEFT] = speed;
    }
    else if (channel == 2)
    {
        targetSpeed_[REAR_RIGHT] = speed;
        outputSpeed_[REAR_RIGHT] = speed;
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
