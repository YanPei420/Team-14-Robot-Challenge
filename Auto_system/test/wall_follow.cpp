#include <Arduino.h>
#include "LidarSensor.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"

namespace {

// ── tuning ────────────────────────────────────────────────────────────
constexpr int16_t FORWARD_SPEED      = 300;   // forward speed (increase if stalls on slope)
constexpr int16_t CORRECTION_SPEED   = 200;   // rotation applied when correcting left/right
constexpr int16_t SEARCH_TURN_SPEED  = 150;   // slow left rotation when searching for wall
constexpr int16_t AVOID_TURN_SPEED   = 300;   // right rotation when obstacle ahead

constexpr float   TARGET_DIST_CM     = 6.0f;  // desired distance from left wall
constexpr float   DIST_DEADBAND_CM   = 2.0f;  // ±2 cm tolerance → 6–10 cm = "within range"
constexpr float   WALL_MAX_DIST_CM   = 50.0f; // beyond this distance = no wall detected
constexpr float   OBSTACLE_DIST_CM   = 30.0f; // front LIDAR threshold for obstacle

constexpr uint32_t LIDAR_TIMEOUT_MS  = 300;
constexpr uint32_t DEBUG_INTERVAL_MS = 200;
// ──────────────────────────────────────────────────────────────────────

LidarSensor  lidarLeft (LIDAR_SERIAL_1);   // left wall
LidarSensor  lidarRight(LIDAR_SERIAL_2);   // right wall (debug only)
LidarSensor  lidarFront(LIDAR_SERIAL_3);   // front obstacle detection

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

enum class State { AVOID, SEARCH, FOLLOW };
State currentState = State::SEARCH;

bool isFresh(LidarSensor& s)
{
    return s.isValid() && (millis() - s.getLastUpdateMs()) < LIDAR_TIMEOUT_MS;
}

bool obstacleAhead()
{
    return isFresh(lidarFront) && lidarFront.getDistanceCM() < OBSTACLE_DIST_CM;
}

bool leftWallVisible()
{
    return isFresh(lidarLeft) && lidarLeft.getDistanceCM() < WALL_MAX_DIST_CM;
}

// Matches the flowchart exactly:
//
//  obstacle ahead?
//    yes → AVOID: rotate right until front is clear
//    no  → left wall detected?
//            no  → SEARCH: rotate left slowly to find wall
//            yes → distance within range?
//                    too close → FOLLOW: forward + steer right (away from wall)
//                    in range  → FOLLOW: forward straight
//                    too far   → FOLLOW: forward + steer left (toward wall)
void wallFollow()
{
    if (obstacleAhead()) {
        currentState = State::AVOID;
        robot.rotate_right(AVOID_TURN_SPEED);
        return;
    }

    if (!leftWallVisible()) {
        currentState = State::SEARCH;
        robot.rotate_left(SEARCH_TURN_SPEED);
        return;
    }

    currentState = State::FOLLOW;
    float dist = lidarLeft.getDistanceCM();

    if (dist < TARGET_DIST_CM - DIST_DEADBAND_CM) {
        // too close to left wall → steer right (positive w = rotate_right)
        robot.drive(FORWARD_SPEED, 0, CORRECTION_SPEED);
    } else if (dist > TARGET_DIST_CM + DIST_DEADBAND_CM) {
        // too far from left wall → steer left (negative w = rotate_left)
        robot.drive(FORWARD_SPEED, 0, -CORRECTION_SPEED);
    } else {
        // within 6–10 cm → go straight
        robot.drive(FORWARD_SPEED, 0, 0);
    }
}

const char* stateName()
{
    switch (currentState) {
        case State::AVOID:  return "AVOID";
        case State::SEARCH: return "SEARCH";
        case State::FOLLOW: return "FOLLOW";
        default:            return "?";
    }
}

void printDebug()
{
    Serial.print("[");
    Serial.print(stateName());
    Serial.print("]  L=");
    Serial.print(isFresh(lidarLeft)  ? lidarLeft.getDistanceCM()  : -1.0f);
    Serial.print("cm  R=");
    Serial.print(isFresh(lidarRight) ? lidarRight.getDistanceCM() : -1.0f);
    Serial.print("cm  F=");
    Serial.print(isFresh(lidarFront) ? lidarFront.getDistanceCM() : -1.0f);
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
    lidarRight.begin();
    lidarFront.begin();

    robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    robot.stop();
}

void loop()
{
    // Get left, right, and front sensor readings
    lidarLeft.update();
    lidarRight.update();
    lidarFront.update();

    wallFollow();

    // LiDAR feedback output (the parallelogram in the flowchart)
    static uint32_t lastDebug = 0;
    if (millis() - lastDebug >= DEBUG_INTERVAL_MS) {
        lastDebug = millis();
        printDebug();
    }
}
