#ifndef IR_CONFIG_H
#define IR_CONFIG_H

#include <Arduino.h>

// ======================================================
// SENSOR ARRAY
// ======================================================

#define IR_SENSOR_COUNT 9

// Pins left to right.
constexpr uint8_t IR_PINS[IR_SENSOR_COUNT] = {45, 46, 47, 48, 49, 50, 51, 52, 53};

// ======================================================
// RC READ TIMING
// ======================================================

constexpr uint16_t IR_CHARGE_TIME_US = 15;
constexpr uint16_t IR_READ_TIMEOUT_US = 1000;

// ======================================================
// SERIAL DEBUG
// ======================================================

#define IR_DEBUG false

#endif
