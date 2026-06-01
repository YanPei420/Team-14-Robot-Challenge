#include <Arduino.h>
#include "LidarSensor.h"
#include "LidarConfig.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"

namespace {

// ── tuning ────────────────────────────────────────────────────────────
constexpr float    OBSTACLE_TRIGGER_CM  = 8.0f;   // front LIDAR threshold
constexpr int16_t  FORWARD_SPEED        = 300;
constexpr int16_t  TURN_SPEED           = 250;     // in-place turn speed
constexpr uint32_t TURN_DURATION_MS     = 600;     // how long to turn before going straight
constexpr uint32_t DEBUG_INTERVAL_MS    = 200;
// ──────────────────────────────────────────────────────────────────────

LidarSensor  lidarLeft (LIDAR_SERIAL_LEFT);
LidarSensor  lidarRight(LIDAR_SERIAL_RIGHT);
LidarSensor  lidarFront(LIDAR_SERIAL_FRONT);
MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

enum class State { FORWARD, TURNING_LEFT, TURNING_RIGHT };
State   currentState  = State::FORWARD;
uint32_t turnStartMs  = 0;

bool isFresh(LidarSensor& s)
{
    return s.isValid()
        && (millis() - s.getLastUpdateMs()) < LIDAR_TIMEOUT_MS;
}

bool frontBlocked()
{
    return isFresh(lidarFront)
        && lidarFront.getDistanceCM() > 0
        && lidarFront.getDistanceCM() < OBSTACLE_TRIGGER_CM;
}

void run()
{
    // ── currently turning — keep turning until TURN_DURATION_MS elapses ──
    if (currentState == State::TURNING_LEFT || currentState == State::TURNING_RIGHT)
    {
        if (millis() - turnStartMs < TURN_DURATION_MS)
        {
            if (currentState == State::TURNING_LEFT)
                robot.rotate_left(TURN_SPEED);
            else
                robot.rotate_right(TURN_SPEED);
            return;
        }
        // Turn complete — resume forward
        currentState = State::FORWARD;
    }

    // ── moving forward ────────────────────────────────────────────────
    if (!frontBlocked())
    {
        robot.forward(FORWARD_SPEED);
        return;
    }

    // ── obstacle detected — compare left vs right ─────────────────────
    float leftDist  = isFresh(lidarLeft)  ? (float)lidarLeft.getDistanceCM()  : 0.0f;
    float rightDist = isFresh(lidarRight) ? (float)lidarRight.getDistanceCM() : 0.0f;

    robot.stop();

    if (leftDist >= rightDist)
    {
        // More space on the left → turn left
        currentState = State::TURNING_LEFT;
        Serial.print("[OBS] L="); Serial.print(leftDist);
        Serial.print(" R="); Serial.print(rightDist);
        Serial.println(" → turning LEFT");
    }
    else
    {
        // More space on the right → turn right
        currentState = State::TURNING_RIGHT;
        Serial.print("[OBS] L="); Serial.print(leftDist);
        Serial.print(" R="); Serial.print(rightDist);
        Serial.println(" → turning RIGHT");
    }

    turnStartMs = millis();
}

void printDebug()
{
    const char* stateName =
        currentState == State::TURNING_LEFT  ? "TURN_L" :
        currentState == State::TURNING_RIGHT ? "TURN_R" : "FWD";

    Serial.print("["); Serial.print(stateName); Serial.print("]");
    Serial.print("  F=");
    if (isFresh(lidarFront)) { Serial.print(lidarFront.getDistanceCM()); Serial.print("cm"); }
    else                     { Serial.print("STALE"); }
    Serial.print("  L=");
    if (isFresh(lidarLeft))  { Serial.print(lidarLeft.getDistanceCM());  Serial.print("cm"); }
    else                     { Serial.print("STALE"); }
    Serial.print("  R=");
    if (isFresh(lidarRight)) { Serial.print(lidarRight.getDistanceCM()); Serial.print("cm"); }
    else                     { Serial.print("STALE"); }
    Serial.println();
}

} // namespace

void setup()
{
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) { ; }
    Serial.println("Obstacle detection started.");

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
    lidarLeft.update();
    lidarRight.update();
    lidarFront.update();

    run();

    static uint32_t lastDebug = 0;
    if (millis() - lastDebug >= DEBUG_INTERVAL_MS) {
        lastDebug = millis();
        printDebug();
    }
}
