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

constexpr uint16_t MOTOR_GEAR_RATIO = 48;
constexpr uint16_t MOTOR_ENCODER_PULSES_PER_MOTOR_REV = 6;
constexpr uint8_t MOTOR_ENCODER_EDGES_PER_PULSE = 4;
constexpr uint16_t MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV =
    MOTOR_ENCODER_PULSES_PER_MOTOR_REV
    * MOTOR_GEAR_RATIO
    * MOTOR_ENCODER_EDGES_PER_PULSE;

// Encoder A/B pins on Arduino GIGA.
// DG01D-E connector order in the reference photo:
// pin 4 = Encoder A, pin 5 = Encoder B.
constexpr uint8_t MOTOR_ENCODER_FRONT_LEFT_A_PIN  = 24; // FL encoder A
constexpr uint8_t MOTOR_ENCODER_FRONT_LEFT_B_PIN  = 25; // FL encoder B
constexpr uint8_t MOTOR_ENCODER_FRONT_RIGHT_A_PIN = 22; // FR encoder A
constexpr uint8_t MOTOR_ENCODER_FRONT_RIGHT_B_PIN = 23; // FR encoder B
constexpr uint8_t MOTOR_ENCODER_REAR_LEFT_A_PIN   = 26; // RL encoder A
constexpr uint8_t MOTOR_ENCODER_REAR_LEFT_B_PIN   = 27; // RL encoder B
constexpr uint8_t MOTOR_ENCODER_REAR_RIGHT_A_PIN  = 28; // RR encoder A
constexpr uint8_t MOTOR_ENCODER_REAR_RIGHT_B_PIN  = 29; // RR encoder B

constexpr int8_t MOTOR_ENCODER_FRONT_LEFT_DIRECTION =
    MOTOR_FRONT_LEFT_DIRECTION;
constexpr int8_t MOTOR_ENCODER_FRONT_RIGHT_DIRECTION =
    MOTOR_FRONT_RIGHT_DIRECTION;
constexpr int8_t MOTOR_ENCODER_REAR_LEFT_DIRECTION =
    MOTOR_REAR_LEFT_DIRECTION;
constexpr int8_t MOTOR_ENCODER_REAR_RIGHT_DIRECTION =
    MOTOR_REAR_RIGHT_DIRECTION;
