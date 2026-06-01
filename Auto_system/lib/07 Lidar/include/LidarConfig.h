#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <Arduino.h>

// ======================================================
// UART — Arduino GIGA R1
// Board labels are offset by 1 from Serial object number.
//   LIDAR 1 TX -> pin 19 (board RX1) -> Serial2  (Left  wall)
//   LIDAR 2 TX -> pin 17 (board RX2) -> Serial3  (Right wall)
//   LIDAR 3 TX -> pin 15 (board RX3) -> Serial4  (Front obstacle)
//   All LIDARs: GND -> GND, 5V -> 5V
// ======================================================

#define LIDAR_SERIAL_LEFT   Serial2
#define LIDAR_SERIAL_RIGHT  Serial3
#define LIDAR_SERIAL_FRONT  Serial4

// Legacy aliases (kept for backwards compatibility with existing code)
#define LIDAR_SERIAL_1      LIDAR_SERIAL_LEFT
#define LIDAR_SERIAL_2      LIDAR_SERIAL_RIGHT
#define LIDAR_SERIAL_3      LIDAR_SERIAL_FRONT
#define LIDAR_SERIAL        LIDAR_SERIAL_LEFT

#define LIDAR_BAUD_RATE     115200

// ======================================================
// FRAME VALIDATION
// ======================================================

#define LIDAR_MIN_STRENGTH      100
#define LIDAR_INVALID_DISTANCE  -1

// Freshness window — TF-Luna outputs ~100 Hz (1 frame / 10 ms).
// A 60 ms timeout = ~6 frames missed before we consider data stale.
#define LIDAR_TIMEOUT_MS        60

// ======================================================
// DEBUG
// ======================================================

// Enables per-frame prints inside LidarSensor (checksum errors, low strength, distance).
// Set false for normal operation.
#define LIDAR_DEBUG             false

#endif