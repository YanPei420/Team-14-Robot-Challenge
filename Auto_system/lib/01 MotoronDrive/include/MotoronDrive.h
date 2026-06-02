#pragma once

#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

#include "MotorConfig.h"
#include "MotorEncoder.h"

struct MotoronSpeedControlConfig
{
    uint16_t intervalMs = MOTOR_SPEED_CONTROL_INTERVAL_MS;
    float maxWheelRPM = MOTOR_SPEED_CONTROL_MAX_WHEEL_RPM;
    float kp = MOTOR_SPEED_CONTROL_KP;
    float ki = MOTOR_SPEED_CONTROL_KI;
    float kd = MOTOR_SPEED_CONTROL_KD;
    float rpmDeadband = MOTOR_SPEED_CONTROL_RPM_DEADBAND;
    float integralLimit = MOTOR_SPEED_CONTROL_INTEGRAL_LIMIT;
    int16_t maxCorrection = MOTOR_SPEED_CONTROL_MAX_CORRECTION;
    int16_t minOutput = MOTOR_SPEED_CONTROL_MIN_OUTPUT;
};

class MotoronDrive
{
private:
    static constexpr uint8_t WHEEL_COUNT = 4;
    static constexpr uint8_t FRONT_LEFT = 0;
    static constexpr uint8_t FRONT_RIGHT = 1;
    static constexpr uint8_t REAR_LEFT = 2;
    static constexpr uint8_t REAR_RIGHT = 3;

    MotoronI2C front_;
    MotoronI2C rear_;
    MotoronDriveEncoders encoders_;

    int16_t maxSpeed_;
    int16_t targetSpeed_[WHEEL_COUNT];
    int16_t outputSpeed_[WHEEL_COUNT];

    MotoronSpeedControlConfig speedControlConfig_;
    bool encoderSpeedControlReady_;
    bool encoderSpeedControlEnabled_;
    uint32_t lastSpeedControlMs_;
    float measuredRPM_[WHEEL_COUNT];
    float targetRPM_[WHEEL_COUNT];
    float speedError_[WHEEL_COUNT];
    float lastSpeedError_[WHEEL_COUNT];
    float speedIntegral_[WHEEL_COUNT];
    float speedCorrection_[WHEEL_COUNT];

    bool distanceMoveActive_;
    bool distanceMoveComplete_;
    uint32_t distanceMoveStartedMs_;
    uint32_t distanceMoveTimeoutMs_;
    int16_t distanceMoveBaseSpeed_;
    int32_t distanceMoveTargetCounts_[WHEEL_COUNT];

    int16_t clampSpeed(int16_t speed) const;
    int16_t applyDirection(int16_t speed, int8_t direction) const;
    void setupController(MotoronI2C& controller);
    void resetEncoderSpeedControlState();
    void resetWheelSpeedControlState(uint8_t wheel);
    void setWheelTarget(uint8_t wheel, int16_t speed);
    void applyTargetSpeeds(bool immediate = false);
    void setTargetSpeeds(
        int16_t frontLeft,
        int16_t frontRight,
        int16_t rearLeft,
        int16_t rearRight
    );
    void writeWheelOutputs(
        int16_t frontLeft,
        int16_t frontRight,
        int16_t rearLeft,
        int16_t rearRight,
        bool immediate = false
    );
    void writeTargetsWithCorrections(bool immediate = false);
    float absFloat(float value) const;
    float targetSpeedToRPM(int16_t speed) const;
    int16_t computeControlledOutput(
        uint8_t wheel,
        int16_t targetSpeed,
        float measuredRPM,
        float dt
    );
    int16_t normalizeDirection(int16_t value) const;
    int16_t absSpeed(int16_t speed) const;
    int32_t absCount(int32_t count) const;
    bool startDistanceMove(
        int16_t vxDirection,
        int16_t vyDirection,
        float distanceCm,
        int16_t speed,
        uint32_t timeoutMs
    );
    void finishDistanceMove(bool complete);
    int16_t scaledDistanceMoveSpeed(int32_t remainingCounts) const;
    void setFrontRaw(int16_t motor1, int16_t motor2, bool immediate);
    void setRearRaw(int16_t motor1, int16_t motor2, bool immediate);

public:
    MotoronDrive(
        uint8_t frontAddress = MOTORON_ADDR_FRONT,
        uint8_t rearAddress = MOTORON_ADDR_REAR
    );

    bool begin();

    void set_max_speed(int16_t maxSpeed);
    int16_t get_max_speed() const;

    bool begin_encoder_speed_control(
        const MotoronSpeedControlConfig& config = MotoronSpeedControlConfig()
    );
    bool set_encoder_speed_control_enabled(bool enabled);
    bool encoder_speed_control_enabled() const;
    bool encoder_speed_control_ready() const;
    bool update_encoder_speed_control();
    bool update();
    void reset_encoder_speed_control();
    void set_encoder_speed_control_config(
        const MotoronSpeedControlConfig& config
    );
    MotoronSpeedControlConfig get_encoder_speed_control_config() const;

    void set_front_left(int16_t speed);
    void set_front_right(int16_t speed);
    void set_rear_left(int16_t speed);
    void set_rear_right(int16_t speed);
    void set_all(
        int16_t frontLeft,
        int16_t frontRight,
        int16_t rearLeft,
        int16_t rearRight
    );
    void get_wheel_speeds(
        int16_t& frontLeft,
        int16_t& frontRight,
        int16_t& rearLeft,
        int16_t& rearRight
    ) const;
    void get_applied_wheel_speeds(
        int16_t& frontLeft,
        int16_t& frontRight,
        int16_t& rearLeft,
        int16_t& rearRight
    ) const;
    void get_encoder_counts(
        int32_t& frontLeft,
        int32_t& frontRight,
        int32_t& rearLeft,
        int32_t& rearRight
    ) const;
    void get_encoder_rpm(
        float& frontLeft,
        float& frontRight,
        float& rearLeft,
        float& rearRight
    ) const;
    void get_target_rpm(
        float& frontLeft,
        float& frontRight,
        float& rearLeft,
        float& rearRight
    ) const;
    int32_t distance_cm_to_encoder_counts(float distanceCm) const;

    bool forward_25cm(
        int16_t speed = MOTOR_DISTANCE_MOVE_DEFAULT_SPEED,
        uint32_t timeoutMs = MOTOR_DISTANCE_MOVE_TIMEOUT_MS
    );
    bool backward_25cm(
        int16_t speed = MOTOR_DISTANCE_MOVE_DEFAULT_SPEED,
        uint32_t timeoutMs = MOTOR_DISTANCE_MOVE_TIMEOUT_MS
    );
    bool left_25cm(
        int16_t speed = MOTOR_DISTANCE_MOVE_DEFAULT_SPEED,
        uint32_t timeoutMs = MOTOR_DISTANCE_MOVE_TIMEOUT_MS
    );
    bool right_25cm(
        int16_t speed = MOTOR_DISTANCE_MOVE_DEFAULT_SPEED,
        uint32_t timeoutMs = MOTOR_DISTANCE_MOVE_TIMEOUT_MS
    );
    bool start_forward_cm(
        float distanceCm,
        int16_t speed = MOTOR_DISTANCE_MOVE_DEFAULT_SPEED,
        uint32_t timeoutMs = MOTOR_DISTANCE_MOVE_TIMEOUT_MS
    );
    bool start_backward_cm(
        float distanceCm,
        int16_t speed = MOTOR_DISTANCE_MOVE_DEFAULT_SPEED,
        uint32_t timeoutMs = MOTOR_DISTANCE_MOVE_TIMEOUT_MS
    );
    bool start_left_cm(
        float distanceCm,
        int16_t speed = MOTOR_DISTANCE_MOVE_DEFAULT_SPEED,
        uint32_t timeoutMs = MOTOR_DISTANCE_MOVE_TIMEOUT_MS
    );
    bool start_right_cm(
        float distanceCm,
        int16_t speed = MOTOR_DISTANCE_MOVE_DEFAULT_SPEED,
        uint32_t timeoutMs = MOTOR_DISTANCE_MOVE_TIMEOUT_MS
    );
    bool update_distance_move();
    bool distance_move_active() const;
    bool distance_move_complete() const;
    void cancel_distance_move();

    void drive(int16_t vx, int16_t vy, int16_t w);
    void forward(int16_t speed);
    void backward(int16_t speed);
    void left(int16_t speed);
    void right(int16_t speed);
    void rotate_left(int16_t speed);
    void rotate_right(int16_t speed);

    void stop();
    void stop_all();

    void raw_front(int16_t motor1, int16_t motor2, bool immediate = false);
    void raw_rear(int16_t motor1, int16_t motor2, bool immediate = false);
    void raw_front_motor(
        uint8_t channel,
        int16_t speed,
        bool immediate = false
    );
    void raw_rear_motor(
        uint8_t channel,
        int16_t speed,
        bool immediate = false
    );

    void clear_status_flags();
    void print_status(Stream& output);
};
