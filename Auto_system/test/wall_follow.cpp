#include <Arduino.h>
#include "LidarSensor.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"
#include "LidarConfig.h"

namespace {

// ── sensor instances ──────────────────────────────────────────────────
LidarSensor  lidarLeft (LIDAR_SERIAL_LEFT);
LidarSensor  lidarFront(LIDAR_SERIAL_FRONT);
MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

// ── state for derivative term + debug ────────────────────────────────
float    lastLeftDist   = WALL_TARGET_CM;
uint32_t lastControlMs  = 0;
int16_t  dbgStrafeCmd   = 0;
int16_t  dbgRotCmd      = 0;
float    dbgError       = 0.0f;
bool     dbgFrontBlock  = false;

// ── helpers ──────────────────────────────────────────────────────────
bool isFresh(LidarSensor& s)
{
    return s.isValid()
        && (millis() - s.getLastUpdateMs()) < LIDAR_TIMEOUT_MS;
}

// Returns true if a usable left-wall reading exists and the wall
// hasn't disappeared (outside corner detection).
bool leftWallVisible(float& distOut)
{
    if (!isFresh(lidarLeft)) return false;
    float d = (float)lidarLeft.getDistanceCM();
    if (d <= 0.0f || d > WALL_LOST_CM) return false;
    distOut = d;
    return true;
}

bool frontObstacleClose()
{
    if (!isFresh(lidarFront)) return false;
    float d = (float)lidarFront.getDistanceCM();
    return (d > 0.0f && d < FRONT_OBSTACLE_CM);
}

// Clamp an int to a symmetric range [-limit, +limit].
int16_t clampSym(float value, int16_t limit)
{
    if (value >  limit) return  limit;
    if (value < -limit) return -limit;
    return (int16_t)value;
}

// ── behaviours ───────────────────────────────────────────────────────

// Inside-corner handling: stop, then pivot right in place until the
// front is clear. Assumes left-hand wall following → corners turn right.
void handleInsideCorner()
{
    robot.drive(0, 0, 0);
    delay(50);  // let inertia settle
    while (frontObstacleClose()) {
        lidarFront.update();
        lidarLeft.update();
        robot.drive(0, 0, CORNER_TURN_SPEED);  // verify sign on your robot
    }
    robot.drive(0, 0, 0);
}

// Outside-corner handling: wall has vanished on the left.
// Curve left (strafe + small forward) to find it again.
void handleOutsideCorner()
{
    robot.drive(FORWARD_SPEED / 2, -MAX_STRAFE_SPEED / 2, 0);
}

// Main P-controlled wall follow using strafe + small yaw.
void wallFollow()
{
    // 1. Inside corner has highest priority.
    dbgFrontBlock = frontObstacleClose();
    if (dbgFrontBlock) {
        handleInsideCorner();
        lastLeftDist  = WALL_TARGET_CM;
        lastControlMs = millis();
        dbgStrafeCmd  = 0;
        dbgRotCmd     = 0;
        dbgError      = 0.0f;
        return;
    }

    // 2. Check left wall.
    float dist;
    if (!leftWallVisible(dist)) {
        handleOutsideCorner();
        dbgStrafeCmd = 0;
        dbgRotCmd    = 0;
        dbgError     = 0.0f;
        return;
    }

    // 3. P-control on lateral error.
    //    error > 0  →  too far from wall  →  strafe LEFT (toward wall)
    //    error < 0  →  too close to wall  →  strafe RIGHT (away)
    float error = dist - WALL_TARGET_CM;
    dbgError = error;

    int16_t strafeCmd = 0;
    if (fabs(error) > WALL_DEADBAND_CM) {
        strafeCmd = clampSym(-KP_STRAFE * error, MAX_STRAFE_SPEED);
    }

    // 4. Small yaw correction from rate of change.
    uint32_t now = millis();
    float dt = (now - lastControlMs) / 1000.0f;
    int16_t rotCmd = 0;
    if (dt > 0.005f && lastControlMs != 0) {
        float deriv = (dist - lastLeftDist) / dt;
        rotCmd = clampSym(-KP_ROTATION * deriv, MAX_ROTATION_SPEED);
    }
    lastLeftDist  = dist;
    lastControlMs = now;
    dbgStrafeCmd  = strafeCmd;
    dbgRotCmd     = rotCmd;

    // 5. Drive (forward, strafe, rotation).
    robot.drive(FORWARD_SPEED, strafeCmd, rotCmd);
}

void printDebug()
{
    // ── distances ────────────────────────────────────────────────────
    Serial.print("L=");
    if (isFresh(lidarLeft)) { Serial.print(lidarLeft.getDistanceCM()); Serial.print("cm"); }
    else                    { Serial.print("STALE"); }

    Serial.print("  F=");
    if (isFresh(lidarFront)) { Serial.print(lidarFront.getDistanceCM()); Serial.print("cm"); }
    else                     { Serial.print("STALE"); }

    // ── control state ────────────────────────────────────────────────
    Serial.print(dbgFrontBlock ? "  [CORNER]" : "  [FOLLOW]");
    Serial.print("  err="); Serial.print(dbgError, 1);
    Serial.print("cm  str="); Serial.print(dbgStrafeCmd);
    Serial.print("  rot=");  Serial.print(dbgRotCmd);

    // ── frame health ─────────────────────────────────────────────────
    Serial.print("  | Lfrm="); Serial.print(lidarLeft.getValidFrameCount());
    Serial.print(" Lchk=");    Serial.print(lidarLeft.getChecksumErrorCount());
    Serial.print(" Ldrp=");    Serial.print(lidarLeft.getDroppedByteCount());
    Serial.print("  Ffrm=");   Serial.print(lidarFront.getValidFrameCount());
    Serial.print(" Fchk=");    Serial.print(lidarFront.getChecksumErrorCount());
    Serial.println();
}

} // namespace

void setup()
{
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) { ; }
    Serial.println("Wall follow (mecanum, P-control) started.");

    lidarLeft.begin();
    lidarFront.begin();

    robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    robot.stop();

    lastControlMs = millis();
}

void loop()
{
    lidarLeft.update();
    lidarFront.update();

    wallFollow();

    static uint32_t lastDebug = 0;
    if (millis() - lastDebug >= DEBUG_INTERVAL_MS) {
        lastDebug = millis();
        printDebug();
    }
}