#ifndef LIDAR_CONFIG_H
#define LIDAR_CONFIG_H

#include <Arduino.h>

// ======================================================
// UART — Arduino GIGA R1
// Board labels are offset by 1 from Serial object number.
//   LIDAR 1 TX -> pin 19 (board RX1) -> Serial2
//   LIDAR 2 TX -> pin 17 (board RX2) -> Serial3
//   LIDAR 3 TX -> pin 15 (board RX3) -> Serial4
//   All LIDARs: GND -> GND, 5V -> 5V
// ======================================================

#define LIDAR_SERIAL_1 Serial2   // Left  wall
#define LIDAR_SERIAL_2 Serial3   // Right wall
#define LIDAR_SERIAL_3 Serial4   // Front obstacle
#define LIDAR_SERIAL_1 Serial2
#define LIDAR_SERIAL_2 Serial3
#define LIDAR_SERIAL_3 Serial4
#define LIDAR_SERIAL   LIDAR_SERIAL_1

#define LIDAR_BAUD_RATE 115200

// ======================================================
// MEASUREMENT
// ======================================================

#define LIDAR_MIN_STRENGTH   100
#define LIDAR_INVALID_DISTANCE -1

// ======================================================
// SERIAL DEBUG
// ======================================================

#define LIDAR_DEBUG          false
#define LIDAR_PRINT_INTERVAL_MS 100

#endif
