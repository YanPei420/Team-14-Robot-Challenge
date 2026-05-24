#pragma once

#include <Arduino.h>

#include "MotorConfig.h"

struct MotorEncoderPins
{
    uint8_t pinA;
    uint8_t pinB;
    int8_t direction;
};

class MotorEncoder
{
private:
    uint8_t pinA_;
    uint8_t pinB_;
    int8_t direction_;
    uint16_t countsPerRevolution_;
    uint8_t slot_;
    volatile int32_t count_;
    volatile uint8_t lastState_;
    int32_t lastSampleCount_;
    uint32_t lastSampleMs_;

    static MotorEncoder* slots_[MOTOR_ENCODER_SLOT_COUNT];

    static void isr0();
    static void isr1();
    static void isr2();
    static void isr3();
    static void dispatch(uint8_t slot);

    uint8_t readState() const;
    int32_t readCountAtomic() const;
    void handleInterrupt();

public:
    MotorEncoder(
        uint8_t pinA = MOTOR_ENCODER_FRONT_LEFT_A_PIN,
        uint8_t pinB = MOTOR_ENCODER_FRONT_LEFT_B_PIN,
        int8_t direction = MOTOR_ENCODER_FRONT_LEFT_DIRECTION,
        uint16_t countsPerRevolution = MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV
    );

    bool begin(
        uint8_t interruptSlot,
        bool usePullups = MOTOR_ENCODER_USE_INTERNAL_PULLUPS
    );

    int32_t read_count() const;
    int32_t read_and_reset();
    void reset_count(int32_t count = 0);

    float read_revolutions() const;
    float sample_rpm();

    uint16_t get_counts_per_revolution() const;
    void set_counts_per_revolution(uint16_t countsPerRevolution);
};

class MotoronDriveEncoders
{
private:
    MotorEncoder frontLeft_;
    MotorEncoder frontRight_;
    MotorEncoder rearLeft_;
    MotorEncoder rearRight_;

public:
    MotoronDriveEncoders(
        MotorEncoderPins frontLeft =
            {
                MOTOR_ENCODER_FRONT_LEFT_A_PIN,
                MOTOR_ENCODER_FRONT_LEFT_B_PIN,
                MOTOR_ENCODER_FRONT_LEFT_DIRECTION
            },
        MotorEncoderPins frontRight =
            {
                MOTOR_ENCODER_FRONT_RIGHT_A_PIN,
                MOTOR_ENCODER_FRONT_RIGHT_B_PIN,
                MOTOR_ENCODER_FRONT_RIGHT_DIRECTION
            },
        MotorEncoderPins rearLeft =
            {
                MOTOR_ENCODER_REAR_LEFT_A_PIN,
                MOTOR_ENCODER_REAR_LEFT_B_PIN,
                MOTOR_ENCODER_REAR_LEFT_DIRECTION
            },
        MotorEncoderPins rearRight =
            {
                MOTOR_ENCODER_REAR_RIGHT_A_PIN,
                MOTOR_ENCODER_REAR_RIGHT_B_PIN,
                MOTOR_ENCODER_REAR_RIGHT_DIRECTION
            },
        uint16_t countsPerRevolution = MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV
    );

    bool begin();
    void reset_counts();

    void get_counts(
        int32_t& frontLeft,
        int32_t& frontRight,
        int32_t& rearLeft,
        int32_t& rearRight
    ) const;

    void get_revolutions(
        float& frontLeft,
        float& frontRight,
        float& rearLeft,
        float& rearRight
    ) const;

    void sample_rpm(
        float& frontLeft,
        float& frontRight,
        float& rearLeft,
        float& rearRight
    );

    MotorEncoder& front_left();
    MotorEncoder& front_right();
    MotorEncoder& rear_left();
    MotorEncoder& rear_right();
};
