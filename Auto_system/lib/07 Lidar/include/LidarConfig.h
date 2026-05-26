#ifndef LIDAR_CONFIG_H
#define LIDAR_CONFIG_H

#include <Arduino.h>
#include <Wire.h>

// ======================================================
// I2C
// ======================================================

// TF-Luna default I2C address is 0x10.
// Motoron front board occupies 0x10, rear board occupies 0x11.
// Change this to 0x12 (or another free address) after reconfiguring
// the sensor via Benewake's PC software or config command.
#define LIDAR_I2C_ADDRESS 0x10

// I2C bus used for the Lidar. Wire1 is reserved for Motoron.
#define LIDAR_WIRE Wire

// ======================================================
// MEASUREMENT
// ======================================================

// Minimum signal strength to treat a reading as valid
#define LIDAR_MIN_STRENGTH 100

// Distance returned when no valid measurement is available (cm)
#define LIDAR_INVALID_DISTANCE -1

// ======================================================
// SERIAL DEBUG
// ======================================================

#define LIDAR_DEBUG false

#endif
