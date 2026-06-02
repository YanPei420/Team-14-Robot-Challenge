#pragma once

#include <Arduino.h>

constexpr uint8_t MOTORON_ADDR_FRONT = 16;
constexpr uint8_t MOTORON_ADDR_REAR = 17;

constexpr uint8_t MOTORON_CHANNEL_FRONT_LEFT = 1;
constexpr uint8_t MOTORON_CHANNEL_FRONT_RIGHT = 2;
constexpr uint8_t MOTORON_CHANNEL_REAR_LEFT = 1;
constexpr uint8_t MOTORON_CHANNEL_REAR_RIGHT = 2;

constexpr int16_t MOTOR_MAX_SPEED = 800;
constexpr uint16_t MOTOR_MAX_ACCELERATION = 300;
constexpr uint16_t MOTOR_MAX_DECELERATION = 600;
constexpr uint16_t MOTOR_COMMAND_TIMEOUT_MS = 1000;

constexpr int8_t MOTOR_FRONT_LEFT_DIRECTION = -1;
constexpr int8_t MOTOR_FRONT_RIGHT_DIRECTION = 1;
constexpr int8_t MOTOR_REAR_LEFT_DIRECTION = -1;
constexpr int8_t MOTOR_REAR_RIGHT_DIRECTION = 1;

constexpr uint8_t MOTOR_ENCODER_SLOT_COUNT = 4;
constexpr bool MOTOR_ENCODER_USE_INTERNAL_PULLUPS = true;
constexpr bool MOTOR_ENCODER_USE_INTERRUPTS = false;

constexpr uint16_t MOTOR_GEAR_RATIO = 48;
constexpr uint16_t MOTOR_ENCODER_PULSES_PER_MOTOR_REV = 6;
constexpr uint8_t MOTOR_ENCODER_EDGES_PER_PULSE = 4;
constexpr uint16_t MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV =
    MOTOR_ENCODER_PULSES_PER_MOTOR_REV
    * MOTOR_GEAR_RATIO
    * MOTOR_ENCODER_EDGES_PER_PULSE;

constexpr float MOTOR_WHEEL_DIAMETER_MM = 65.0f;
constexpr float MOTOR_WHEEL_CIRCUMFERENCE_MM =
    MOTOR_WHEEL_DIAMETER_MM * 3.14159265f;

// Encoder A/B pins on Arduino GIGA.
// DG01D-E connector order in the reference photo:
// pin 4 = Encoder A, pin 5 = Encoder B.
constexpr uint8_t MOTOR_ENCODER_FRONT_LEFT_A_PIN  = 24; // FL encoder A
constexpr uint8_t MOTOR_ENCODER_FRONT_LEFT_B_PIN  = 25; // FL encoder B
constexpr uint8_t MOTOR_ENCODER_FRONT_RIGHT_A_PIN = 22; // FR encoder A
constexpr uint8_t MOTOR_ENCODER_FRONT_RIGHT_B_PIN = 23; // FR encoder B
constexpr uint8_t MOTOR_ENCODER_REAR_LEFT_A_PIN   = 28; // RL encoder A
constexpr uint8_t MOTOR_ENCODER_REAR_LEFT_B_PIN   = 29; // RL encoder B
constexpr uint8_t MOTOR_ENCODER_REAR_RIGHT_A_PIN  = 26; // RR encoder A
constexpr uint8_t MOTOR_ENCODER_REAR_RIGHT_B_PIN  = 27; // RR encoder B

constexpr int8_t MOTOR_ENCODER_FRONT_LEFT_DIRECTION =
    MOTOR_FRONT_LEFT_DIRECTION;
constexpr int8_t MOTOR_ENCODER_FRONT_RIGHT_DIRECTION =
    MOTOR_FRONT_RIGHT_DIRECTION;
constexpr int8_t MOTOR_ENCODER_REAR_LEFT_DIRECTION =
    MOTOR_REAR_LEFT_DIRECTION;
constexpr int8_t MOTOR_ENCODER_REAR_RIGHT_DIRECTION =
    MOTOR_REAR_RIGHT_DIRECTION;

// ======================================================
// ENCODER SPEED CONTROL
// ======================================================

constexpr uint16_t MOTOR_SPEED_CONTROL_INTERVAL_MS = 50;
constexpr bool MOTOR_SPEED_CONTROL_AUTO_BEGIN = true;
constexpr float MOTOR_SPEED_CONTROL_MAX_WHEEL_RPM = 180.0f;
constexpr float MOTOR_SPEED_CONTROL_KP = 2.0f;
constexpr float MOTOR_SPEED_CONTROL_KI = 0.25f;
constexpr float MOTOR_SPEED_CONTROL_KD = 0.0f;
constexpr float MOTOR_SPEED_CONTROL_RPM_DEADBAND = 2.0f;
constexpr float MOTOR_SPEED_CONTROL_INTEGRAL_LIMIT = 300.0f;
constexpr int16_t MOTOR_SPEED_CONTROL_MAX_CORRECTION = 300;
constexpr int16_t MOTOR_SPEED_CONTROL_MIN_OUTPUT = 60;

// ======================================================
// ENCODER DISTANCE MOVES
// ======================================================

constexpr float MOTOR_DISTANCE_MOVE_25CM = 25.0f;
constexpr int16_t MOTOR_DISTANCE_MOVE_DEFAULT_SPEED = 220;
constexpr int16_t MOTOR_DISTANCE_MOVE_MIN_SPEED = 80;
constexpr uint16_t MOTOR_DISTANCE_MOVE_TOLERANCE_COUNTS = 24;
constexpr uint16_t MOTOR_DISTANCE_MOVE_SLOWDOWN_COUNTS = 260;
constexpr uint32_t MOTOR_DISTANCE_MOVE_TIMEOUT_MS = 5000;
