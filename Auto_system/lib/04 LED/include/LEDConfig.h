#ifndef LED_CONFIG_H
#define LED_CONFIG_H

#include <Arduino.h>

// ======================================================
// LED PINS
// ======================================================

constexpr uint8_t LED_GREEN_PIN = 32;
constexpr uint8_t LED_RED_PIN = 34;

// ======================================================
// FLASH CONTROL
// ======================================================

constexpr unsigned long LED_RED_FLASH_INTERVAL_MS = 300;

#endif
