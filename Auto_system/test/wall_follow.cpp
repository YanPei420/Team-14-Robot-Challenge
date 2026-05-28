#include <Arduino.h>
#include "LidarSensor.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"

namespace {

// ── tuning ────────────────────────────────────────────────────────────
constexpr int16_t FORWARD_SPEED     = 300;   // forward speed (increase if stalls on slope)
constexpr int16_t CORRECTION_SPEED  = 200;   // rotation correction when off-target

constexpr float   TARGET_DIST_CM    = 6.0f;  // desired distance from left wall
constexpr float   DIST_DEADBAND_CM  = 2.0f;  // ±2 cm tolerance → 4–8 cm = "in range"

constexpr uint32_t LIDAR_TIMEOUT_MS  = 300;
constexpr uint32_t DEBUG_INTERVAL_MS = 200;
// ──────────────────────────────────────────────────────────────────────

LidarSensor  lidarLeft(LIDAR_SERIAL_1);   // left wall
MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

bool leftFresh()
{
    return lidarLeft.isValid()
        && (millis() - lidarLeft.getLastUpdateMs()) < LIDAR_TIMEOUT_MS;
}

void wallFollow()
{
    if (!leftFresh()) {
        // No left wall reading — just go straight
        robot.drive(FORWARD_SPEED, 0, 0);
        return;
    }

    float dist = lidarLeft.getDistanceCM();

    if (dist < TARGET_DIST_CM - DIST_DEADBAND_CM) {
        // Too close → steer right (away from wall)
        robot.drive(FORWARD_SPEED, 0, CORRECTION_SPEED);
    } else if (dist > TARGET_DIST_CM + DIST_DEADBAND_CM) {
        // Too far → steer left (toward wall)
        robot.drive(FORWARD_SPEED, 0, -CORRECTION_SPEED);
    } else {
        // Within 4–8 cm → go straight
        robot.drive(FORWARD_SPEED, 0, 0);
    }
}

void printDebug()
{
    Serial.print("L=");
    if (leftFresh()) Serial.print(lidarLeft.getDistanceCM());
    else             Serial.print("--");
    Serial.println("cm");
}

} // namespace

void setup()
{
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) { ; }
    Serial.println("Wall follow started.");

    lidarLeft.begin();

    robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    robot.stop();
}

void loop()
{
    lidarLeft.update();
    wallFollow();

    static uint32_t lastDebug = 0;
    if (millis() - lastDebug >= DEBUG_INTERVAL_MS) {
        lastDebug = millis();
        printDebug();
    }
}
