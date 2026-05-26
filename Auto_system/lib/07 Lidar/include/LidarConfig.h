#ifndef LIDAR_CONFIG_H
#define LIDAR_CONFIG_H

#include <Arduino.h>

// ======================================================
// UART
// ! blue to TX0, green to RX0 on Arduino Uno (pins 0 and 1)
// ======================================================

// Serial port connected to TF-Luna (TX->RX1, RX->TX1)
#define LIDAR_SERIAL Serial1

// TF-Luna default baud rate
#define LIDAR_BAUD_RATE 115200

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
