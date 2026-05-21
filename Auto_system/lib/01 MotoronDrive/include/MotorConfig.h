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

constexpr int8_t MOTOR_FRONT_LEFT_DIRECTION = 1;
constexpr int8_t MOTOR_FRONT_RIGHT_DIRECTION = -1;
constexpr int8_t MOTOR_REAR_LEFT_DIRECTION = 1;
constexpr int8_t MOTOR_REAR_RIGHT_DIRECTION = -1;
