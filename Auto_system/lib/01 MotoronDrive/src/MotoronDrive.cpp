#include "MotoronDrive.h"
#include "MotorConfig.h"
#include "MotoronChassis.h"

MotoronDrive::MotoronDrive(uint8_t Front_Board, uint8_t Rear_Board)
    : front_(Front_Board),
      rear_(Rear_Board),
      maxSpeed_(MOTOR_MAX_SPEED),
      currentFrontLeft_(0),
      currentFrontRight_(0),
      currentRearLeft_(0),
      currentRearRight_(0)
{
}

void MotoronDrive::begin()
{
    Wire1.begin();

    setup_controller(front_);
    setup_controller(rear_);

    stop_all();

    Serial.println("MotoronDrive initialized");
}

void MotoronDrive::setup_controller(MotoronI2C& controller)
{
    controller.setBus(&Wire1);

    controller.reinitialize();
    controller.disableCrc();
    controller.clearResetFlag();
    controller.disableCommandTimeout();
    controller.clearMotorFaultUnconditional();

    configure_motor_limits(controller, 1);
    configure_motor_limits(controller, 2);
}

void MotoronDrive::configure_motor_limits(MotoronI2C& controller, uint8_t motor)
{
    controller.setMaxAcceleration(motor, MOTORON_MAX_ACCELERATION);
    controller.setMaxDeceleration(motor, MOTORON_MAX_DECELERATION);
}

void MotoronDrive::stop()
{
    stop_all();
}

void MotoronDrive::stop_all()
{
    set_all(0, 0, 0, 0);
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
    currentFrontLeft_ = clamp_speed(fl);
    currentFrontRight_ = clamp_speed(fr);
    currentRearLeft_ = clamp_speed(rl);
    currentRearRight_ = clamp_speed(rr);

    front_.setSpeed(MOTOR_FL, currentFrontLeft_);
    front_.setSpeed(MOTOR_FR, currentFrontRight_);
    rear_.setSpeed(MOTOR_RL, currentRearLeft_);
    rear_.setSpeed(MOTOR_RR, currentRearRight_);
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
    MotoronChassis::drive(*this, vx, vy, w);
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

int16_t MotoronDrive::clamp_speed(int16_t speed) const
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

void MotoronDrive::raw_front(int16_t motor1, int16_t motor2)
{
    front_.setSpeed(1, clamp_speed(motor1));
    front_.setSpeed(2, clamp_speed(motor2));
}

void MotoronDrive::raw_rear(int16_t motor1, int16_t motor2)
{
    rear_.setSpeed(1, clamp_speed(motor1));
    rear_.setSpeed(2, clamp_speed(motor2));
}

void MotoronDrive::raw_front_motor(uint8_t motor, int16_t speed, bool immediate)
{
    if (immediate)
    {
        front_.setSpeedNow(motor, clamp_speed(speed));
        return;
    }

    front_.setSpeed(motor, clamp_speed(speed));
}

void MotoronDrive::raw_rear_motor(uint8_t motor, int16_t speed, bool immediate)
{
    if (immediate)
    {
        rear_.setSpeedNow(motor, clamp_speed(speed));
        return;
    }

    rear_.setSpeed(motor, clamp_speed(speed));
}

void MotoronDrive::resend_current_speeds()
{
    set_all(currentFrontLeft_, currentFrontRight_, currentRearLeft_, currentRearRight_);
}

void MotoronDrive::clear_status_flags()
{
    front_.clearResetFlag();
    front_.clearMotorFaultUnconditional();
    rear_.clearResetFlag();
    rear_.clearMotorFaultUnconditional();
}

void MotoronDrive::print_status(Stream& output)
{
    print_controller_status(output, "front", front_);
    print_controller_status(output, "rear", rear_);
}

void MotoronDrive::clear_controller_status_flags(MotoronI2C& controller)
{
    controller.clearResetFlag();
    controller.clearMotorFaultUnconditional();
}

void MotoronDrive::print_controller_status(Stream& output, const char* name, MotoronI2C& controller)
{
    const uint16_t statusFlags = controller.getStatusFlags();

    output.print(name);
    output.print(" addr=0x");
    output.print(controller.getAddress(), HEX);
    output.print(" lastError=");
    output.print(controller.getLastError());
    output.print(" status=0x");
    output.print(statusFlags, HEX);
    output.print(" errorActive=");
    output.print((statusFlags & (1 << MOTORON_STATUS_FLAG_ERROR_ACTIVE)) ? "1" : "0");
    output.print(" outputEnabled=");
    output.print((statusFlags & (1 << MOTORON_STATUS_FLAG_MOTOR_OUTPUT_ENABLED)) ? "1" : "0");
    output.print(" motorDriving=");
    output.print((statusFlags & (1 << MOTORON_STATUS_FLAG_MOTOR_DRIVING)) ? "1" : "0");
    output.print(" noPower=");
    output.print((statusFlags & (1 << MOTORON_STATUS_FLAG_NO_POWER)) ? "1" : "0");
    output.print(" fault=");
    output.println((statusFlags & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULTING)) ? "1" : "0");

    print_motor_status(output, controller, 1);
    print_motor_status(output, controller, 2);
}

void MotoronDrive::print_motor_status(Stream& output, MotoronI2C& controller, uint8_t motor)
{
    output.print("  M");
    output.print(motor);
    output.print(" target=");
    output.print(controller.getTargetSpeed(motor));
    output.print(" current=");
    output.println(controller.getCurrentSpeed(motor));
}
