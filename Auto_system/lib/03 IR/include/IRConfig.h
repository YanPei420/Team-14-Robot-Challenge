#ifndef IR_CONFIG_H
#define IR_CONFIG_H

#include <Arduino.h>

// ======================================================
// SENSOR ARRAY
// ======================================================

#define IR_SENSOR_COUNT 5

// Pins left to right: far-left, left, center, right, far-right
constexpr uint8_t IR_PINS[IR_SENSOR_COUNT] = {40, 41, 42, 43, 44};

// ======================================================
// LINE LOGIC
// ======================================================

// HIGH = line detected
// LOW  = no line
#define IR_LINE_STATE HIGH

// ======================================================
// SERIAL DEBUG
// ======================================================

#define IR_DEBUG false

#endif
