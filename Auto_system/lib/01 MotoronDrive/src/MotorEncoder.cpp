#include "MotorEncoder.h"

namespace
{
constexpr int8_t QUADRATURE_TRANSITION_TABLE[16] =
{
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};
}

MotorEncoder* MotorEncoder::slots_[MOTOR_ENCODER_SLOT_COUNT] =
{
    nullptr,
    nullptr,
    nullptr,
    nullptr
};

MotorEncoder::MotorEncoder(
    uint8_t pinA,
    uint8_t pinB,
    int8_t direction,
    uint16_t countsPerRevolution
)
    : pinA_(pinA),
      pinB_(pinB),
      direction_(direction < 0 ? -1 : 1),
      countsPerRevolution_(countsPerRevolution),
      slot_(MOTOR_ENCODER_SLOT_COUNT),
      count_(0),
      lastState_(0),
      lastSampleCount_(0),
      lastSampleMs_(0)
{
}

bool MotorEncoder::begin(uint8_t interruptSlot, bool usePullups)
{
    if (interruptSlot >= MOTOR_ENCODER_SLOT_COUNT)
    {
        return false;
    }

    const int interruptA = digitalPinToInterrupt(pinA_);
    const int interruptB = digitalPinToInterrupt(pinB_);

    if (interruptA == NOT_AN_INTERRUPT || interruptB == NOT_AN_INTERRUPT)
    {
        return false;
    }

    pinMode(pinA_, usePullups ? INPUT_PULLUP : INPUT);
    pinMode(pinB_, usePullups ? INPUT_PULLUP : INPUT);

    noInterrupts();
    slot_ = interruptSlot;
    count_ = 0;
    lastState_ = readState();
    lastSampleCount_ = 0;
    lastSampleMs_ = millis();
    slots_[interruptSlot] = this;
    interrupts();

    switch (interruptSlot)
    {
        case 0:
            attachInterrupt(interruptA, MotorEncoder::isr0, CHANGE);
            attachInterrupt(interruptB, MotorEncoder::isr0, CHANGE);
            break;
        case 1:
            attachInterrupt(interruptA, MotorEncoder::isr1, CHANGE);
            attachInterrupt(interruptB, MotorEncoder::isr1, CHANGE);
            break;
        case 2:
            attachInterrupt(interruptA, MotorEncoder::isr2, CHANGE);
            attachInterrupt(interruptB, MotorEncoder::isr2, CHANGE);
            break;
        case 3:
            attachInterrupt(interruptA, MotorEncoder::isr3, CHANGE);
            attachInterrupt(interruptB, MotorEncoder::isr3, CHANGE);
            break;
        default:
            return false;
    }

    return true;
}

int32_t MotorEncoder::read_count() const
{
    return readCountAtomic();
}

int32_t MotorEncoder::read_and_reset()
{
    const uint32_t now = millis();

    noInterrupts();
    const int32_t oldCount = count_;
    count_ = 0;
    lastSampleCount_ = 0;
    lastSampleMs_ = now;
    interrupts();

    return oldCount;
}

void MotorEncoder::reset_count(int32_t count)
{
    const uint32_t now = millis();

    noInterrupts();
    count_ = count;
    lastSampleCount_ = count;
    lastSampleMs_ = now;
    interrupts();
}

float MotorEncoder::read_revolutions() const
{
    if (countsPerRevolution_ == 0)
    {
        return 0.0f;
    }

    return static_cast<float>(readCountAtomic()) / countsPerRevolution_;
}

float MotorEncoder::sample_rpm()
{
    const uint32_t now = millis();
    const int32_t currentCount = readCountAtomic();
    const uint32_t elapsedMs = now - lastSampleMs_;

    if (elapsedMs == 0 || countsPerRevolution_ == 0)
    {
        return 0.0f;
    }

    const int32_t delta = currentCount - lastSampleCount_;
    lastSampleCount_ = currentCount;
    lastSampleMs_ = now;

    return (static_cast<float>(delta) * 60000.0f)
        / (static_cast<float>(countsPerRevolution_) * elapsedMs);
}

uint16_t MotorEncoder::get_counts_per_revolution() const
{
    return countsPerRevolution_;
}

void MotorEncoder::set_counts_per_revolution(uint16_t countsPerRevolution)
{
    countsPerRevolution_ = countsPerRevolution;
}

void MotorEncoder::isr0()
{
    dispatch(0);
}

void MotorEncoder::isr1()
{
    dispatch(1);
}

void MotorEncoder::isr2()
{
    dispatch(2);
}

void MotorEncoder::isr3()
{
    dispatch(3);
}

void MotorEncoder::dispatch(uint8_t slot)
{
    if (slot < MOTOR_ENCODER_SLOT_COUNT && slots_[slot] != nullptr)
    {
        slots_[slot]->handleInterrupt();
    }
}

uint8_t MotorEncoder::readState() const
{
    return (digitalRead(pinA_) ? 0x02 : 0x00)
        | (digitalRead(pinB_) ? 0x01 : 0x00);
}

int32_t MotorEncoder::readCountAtomic() const
{
    noInterrupts();
    const int32_t count = count_;
    interrupts();

    return count;
}

void MotorEncoder::handleInterrupt()
{
    const uint8_t state = readState();
    const uint8_t transition = (lastState_ << 2) | state;

    count_ += QUADRATURE_TRANSITION_TABLE[transition] * direction_;
    lastState_ = state;
}

MotoronDriveEncoders::MotoronDriveEncoders(
    MotorEncoderPins frontLeft,
    MotorEncoderPins frontRight,
    MotorEncoderPins rearLeft,
    MotorEncoderPins rearRight,
    uint16_t countsPerRevolution
)
    : frontLeft_(
          frontLeft.pinA,
          frontLeft.pinB,
          frontLeft.direction,
          countsPerRevolution
      ),
      frontRight_(
          frontRight.pinA,
          frontRight.pinB,
          frontRight.direction,
          countsPerRevolution
      ),
      rearLeft_(
          rearLeft.pinA,
          rearLeft.pinB,
          rearLeft.direction,
          countsPerRevolution
      ),
      rearRight_(
          rearRight.pinA,
          rearRight.pinB,
          rearRight.direction,
          countsPerRevolution
      )
{
}

bool MotoronDriveEncoders::begin()
{
    bool ok = true;

    ok = frontLeft_.begin(0) && ok;
    ok = frontRight_.begin(1) && ok;
    ok = rearLeft_.begin(2) && ok;
    ok = rearRight_.begin(3) && ok;

    return ok;
}

void MotoronDriveEncoders::reset_counts()
{
    frontLeft_.reset_count();
    frontRight_.reset_count();
    rearLeft_.reset_count();
    rearRight_.reset_count();
}

void MotoronDriveEncoders::get_counts(
    int32_t& frontLeft,
    int32_t& frontRight,
    int32_t& rearLeft,
    int32_t& rearRight
) const
{
    frontLeft = frontLeft_.read_count();
    frontRight = frontRight_.read_count();
    rearLeft = rearLeft_.read_count();
    rearRight = rearRight_.read_count();
}

void MotoronDriveEncoders::get_revolutions(
    float& frontLeft,
    float& frontRight,
    float& rearLeft,
    float& rearRight
) const
{
    frontLeft = frontLeft_.read_revolutions();
    frontRight = frontRight_.read_revolutions();
    rearLeft = rearLeft_.read_revolutions();
    rearRight = rearRight_.read_revolutions();
}

void MotoronDriveEncoders::sample_rpm(
    float& frontLeft,
    float& frontRight,
    float& rearLeft,
    float& rearRight
)
{
    frontLeft = frontLeft_.sample_rpm();
    frontRight = frontRight_.sample_rpm();
    rearLeft = rearLeft_.sample_rpm();
    rearRight = rearRight_.sample_rpm();
}

MotorEncoder& MotoronDriveEncoders::front_left()
{
    return frontLeft_;
}

MotorEncoder& MotoronDriveEncoders::front_right()
{
    return frontRight_;
}

MotorEncoder& MotoronDriveEncoders::rear_left()
{
    return rearLeft_;
}

MotorEncoder& MotoronDriveEncoders::rear_right()
{
    return rearRight_;
}
