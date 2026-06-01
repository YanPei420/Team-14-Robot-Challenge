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
// WALL-FOLLOWING GEOMETRY
// ======================================================

// Desired lateral distance from the followed wall (cm).
#define WALL_TARGET_CM          6.0f

// Deadband — within ±this many cm we consider the robot "on track"
// and stop applying lateral correction.
#define WALL_DEADBAND_CM        1.0f

// If the left LIDAR reads beyond this, we assume the wall has
// disappeared (outside corner) rather than trusting the value.
#define WALL_LOST_CM            40.0f

// Front LIDAR threshold — when an obstacle is closer than this,
// the robot is at an inside corner / dead end and must turn.
#define FRONT_OBSTACLE_CM       12.0f

// ======================================================
// MOTION TUNING (mecanum)
// ======================================================

#define FORWARD_SPEED           300    // base forward velocity
#define MAX_STRAFE_SPEED        250    // cap on lateral correction
#define MAX_ROTATION_SPEED      180    // cap on yaw correction

// Proportional gains.
// Strafe handles "I'm too close / too far" laterally.
// Rotation handles "I'm pointing into / away from the wall".
#define KP_STRAFE               40.0f  // (cm error) -> strafe cmd
#define KP_ROTATION             8.0f   // (cm error derivative) -> yaw cmd

// In-place turn speed used at inside corners.
#define CORNER_TURN_SPEED       220

// ======================================================
// DEBUG
// ======================================================

// Enables per-frame prints inside LidarSensor (checksum errors, low strength, distance).
// Set false for normal operation — generates a lot of serial output.
#define LIDAR_DEBUG             false

#define DEBUG_INTERVAL_MS       200

#endif