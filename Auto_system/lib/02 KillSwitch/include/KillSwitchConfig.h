#ifndef KILL_SWITCH_CONFIG_H
#define KILL_SWITCH_CONFIG_H

#include <Arduino.h>

// ======================================================
// BUTTON PIN
// ======================================================

constexpr uint8_t KILL_SWITCH_PIN = 33;

// ======================================================
// BUTTON STATE
// ======================================================

constexpr uint8_t KILL_SWITCH_ACTIVE_STATE = LOW;

// ======================================================
// SERIAL DEBUG
// ======================================================

constexpr bool KILL_SWITCH_DEBUG = false;

#endif
