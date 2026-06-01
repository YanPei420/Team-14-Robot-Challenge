#include <Arduino.h>
#include "LidarSensor.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"
#include "LidarConfig.h"

namespace {

// ── sensor instances ──────────────────────────────────────────────────
LidarSensor  lidarLeft (LIDAR_SERIAL_LEFT);
LidarSensor  lidarRight(LIDAR_SERIAL_RIGHT);
LidarSensor  lidarFront(LIDAR_SERIAL_FRONT);
MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

// ── state ────────────────────────────────────────────────────────────
// State machine — keeps logic explicit and debuggable.
enum AvoidState {
    DRIVING_FORWARD,
    TURNING_LEFT,
    TURNING_RIGHT,
    UTURNING
};

AvoidState state = DRIVING_FORWARD;

// ── helpers ──────────────────────────────────────────────────────────
bool isFresh(LidarSensor& s)
{
    return s.isValid()
        && (millis() - s.getLastUpdateMs()) < LIDAR_TIMEOUT_MS;
}

// Read a LIDAR distance, returning a large sentinel value if invalid
// or stale. Using a large value (rather than -1) means downstream
// "is this side clear?" checks naturally treat invalid as "clear" —
// which is the safer assumption for SIDE sensors (we'd rather attempt
// a turn into uncertain space than commit to a U-turn unnecessarily).
// For FRONT sensor we treat invalid as "blocked" instead — see below.
float readSideDistance(LidarSensor& s)
{
    if (!isFresh(s)) return 1000.0f;        // treat unknown side as open
    int16_t raw = s.getDistanceCM();
    if (raw <= 0) return 1000.0f;
    return (float)raw;
}

// Front sensor: treat invalid as BLOCKED (small value).
// Safer to stop/turn on uncertainty than to plow forward blind.
float readFrontDistance()
{
    if (!isFresh(lidarFront)) return 0.0f;  // treat unknown front as blocked
    int16_t raw = lidarFront.getDistanceCM();
    if (raw <= 0) return 0.0f;
    return (float)raw;
}

void updateAllLidars()
{
    lidarLeft.update();
    lidarRight.update();
    lidarFront.update();
}

// ── decision logic ───────────────────────────────────────────────────

// Called when front becomes blocked. Decides which way to turn,
// or whether to U-turn.
AvoidState chooseAvoidDirection()
{
    float left  = readSideDistance(lidarLeft);
    float right = readSideDistance(lidarRight);

    bool leftClear  = (left  > SIDE_MIN_CLEAR_CM);
    bool rightClear = (right > SIDE_MIN_CLEAR_CM);

    // Both sides blocked → U-turn.
    if (!leftClear && !rightClear) {
        Serial.println("[avoid] both sides blocked -> U-turn");
        return UTURNING;
    }

    // Only one side clear → take it.
    if (leftClear && !rightClear) {
        Serial.println("[avoid] only left clear -> turn left");
        return TURNING_LEFT;
    }
    if (!leftClear && rightClear) {
        Serial.println("[avoid] only right clear -> turn right");
        return TURNING_RIGHT;
    }

    // Both sides clear → pick the wider one, with tie-breaker.
    float diff = left - right;
    if (fabs(diff) < SIDE_TIE_THRESHOLD_CM) {
        Serial.println("[avoid] tie -> turn right (default)");
        return TURNING_RIGHT;          // tiebreaker
    }
    if (diff > 0) {
        Serial.print("[avoid] left wider (L=");
        Serial.print(left); Serial.print(" R="); Serial.print(right);
        Serial.println(") -> turn left");
        return TURNING_LEFT;
    } else {
        Serial.print("[avoid] right wider (L=");
        Serial.print(left); Serial.print(" R="); Serial.print(right);
        Serial.println(") -> turn right");
        return TURNING_RIGHT;
    }
}

// ── state actions ────────────────────────────────────────────────────
// Each state runs one iteration of `loop()`. The state machine
// is checked again next iteration — this keeps things non-blocking
// so LIDAR updates and debug prints keep flowing.

void runDrivingForward()
{
    float front = readFrontDistance();

    if (front < OBSTACLE_TRIGGER_CM) {
        robot.drive(0, 0, 0);           // stop before deciding
        state = chooseAvoidDirection();
        return;
    }

    robot.drive(FORWARD_SPEED, 0, 0);
}

void runTurningLeft()
{
    float front = readFrontDistance();

    // Hysteresis: only resume forward when comfortably clear.
    if (front > OBSTACLE_CLEAR_CM) {
        robot.drive(0, 0, 0);
        Serial.println("[avoid] front clear -> resume forward");
        state = DRIVING_FORWARD;
        return;
    }

    // Pivot left in place.
    // NOTE: sign convention — verify on robot.
    // If positive rotation = CW (right), use -AVOID_TURN_SPEED here.
    robot.drive(0, 0, -AVOID_TURN_SPEED);
}

void runTurningRight()
{
    float front = readFrontDistance();

    if (front > OBSTACLE_CLEAR_CM) {
        robot.drive(0, 0, 0);
        Serial.println("[avoid] front clear -> resume forward");
        state = DRIVING_FORWARD;
        return;
    }

    robot.drive(0, 0, AVOID_TURN_SPEED);
}

void runUturning()
{
    float front = readFrontDistance();
    float left  = readSideDistance(lidarLeft);
    float right = readSideDistance(lidarRight);

    // U-turn complete when front is clear AND at least one side has
    // opened up — this prevents stopping mid-spin while still facing
    // a different part of the same dead-end wall.
    bool frontOpen = (front > OBSTACLE_CLEAR_CM);
    bool sideOpen  = (left > SIDE_MIN_CLEAR_CM) || (right > SIDE_MIN_CLEAR_CM);

    if (frontOpen && sideOpen) {
        robot.drive(0, 0, 0);
        Serial.println("[avoid] U-turn complete -> resume forward");
        state = DRIVING_FORWARD;
        return;
    }

    // Spin in place (direction: right, by convention).
    robot.drive(0, 0, UTURN_SPEED);
}

// ── debug ────────────────────────────────────────────────────────────
const char* stateName(AvoidState s)
{
    switch (s) {
        case DRIVING_FORWARD: return "FWD";
        case TURNING_LEFT:    return "TL";
        case TURNING_RIGHT:   return "TR";
        case UTURNING:        return "UTURN";
    }
    return "?";
}

void printDebug()
{
    Serial.print("["); Serial.print(stateName(state)); Serial.print("] ");

    Serial.print("L=");
    if (isFresh(lidarLeft))  Serial.print(lidarLeft.getDistanceCM());
    else                     Serial.print("--");

    Serial.print(" F=");
    if (isFresh(lidarFront)) Serial.print(lidarFront.getDistanceCM());
    else                     Serial.print("--");

    Serial.print(" R=");
    if (isFresh(lidarRight)) Serial.print(lidarRight.getDistanceCM());
    else                     Serial.print("--");

    Serial.println("cm");
}

} // namespace

void setup()
{
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) { ; }
    Serial.println("Obstacle avoidance started.");

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
    updateAllLidars();

    switch (state) {
        case DRIVING_FORWARD: runDrivingForward(); break;
        case TURNING_LEFT:    runTurningLeft();    break;
        case TURNING_RIGHT:   runTurningRight();   break;
        case UTURNING:        runUturning();       break;
    }

    static uint32_t lastDebug = 0;
    if (millis() - lastDebug >= DEBUG_INTERVAL_MS) {
        lastDebug = millis();
        printDebug();
    }
}