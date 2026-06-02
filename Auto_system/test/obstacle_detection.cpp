// Obstacle bypass during line following.
// On front obstacle + RFID: reverse, turn left, line follow along obstacle edge
// (watching Serial2/lidarLeft for sharp increase), turn right twice to resume
// original direction.

#include <Arduino.h>
#include "IRConfig.h"
#include "IRSensor.h"
#include "LineFollower.h"
#include "LidarSensor.h"
#include "LidarConfig.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"
#include "RFIDHandler.h"

namespace {

// --- tuning ---
constexpr int16_t  DRIVE_SPEED       = 220;
constexpr int16_t  TURN_SPEED        = 260;
constexpr int16_t  REVERSE_SPEED     = 180;
constexpr uint32_t TURN_MS           = 700;
constexpr uint32_t REVERSE_MS        = 400;
constexpr uint32_t RFID_COOLDOWN_MS  = 1500;
constexpr uint32_t RFID_WINDOW_MS    = 2000;
constexpr uint32_t LIDAR_FRESH_MS    = 300;
constexpr float    OBSTACLE_CM       = 30.0f;
constexpr float    EDGE_NEAR_CM      = 20.0f;  // lidar was close to obstacle
constexpr float    EDGE_FAR_CM       = 35.0f;  // lidar jumped to this = edge passed
constexpr uint32_t DEBUG_INTERVAL_MS = 300;
// ---

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
IRSensor     lineSensors(IR_PINS, IR_SENSOR_COUNT);
LineFollower lineFollower(robot, lineSensors);
LidarSensor  lidarFront(LIDAR_SERIAL_FRONT);   // Serial4 - obstacle detection
LidarSensor  lidarLeft (LIDAR_SERIAL_LEFT);    // Serial2 - edge detection during bypass
RFIDHandler  rfid;

enum class Step : uint8_t {
    Forward,           // normal line follow
    Reverse,           // back up after obstacle
    TurnLeft,          // turn left 90 deg
    BypassLineFollow,  // line follow perpendicular, watch lidarLeft for edge+rfid
    BypassTurnRight1,  // first right turn 90 deg
    BypassForward,     // line follow in new direction, watch lidarLeft for edge
    BypassTurnRight2,  // second right turn 90 deg
    FindLine,          // rotate until line found before resuming
    Resume,            // back to original direction, line follow
};

Step     step         = Step::Forward;
bool     armed        = false;
bool     motorReady   = false;
bool     encReady     = false;
uint32_t turnStartMs  = 0;
uint32_t lastRfidMs   = 0;
uint32_t lastDebugMs  = 0;
bool     rfidThisLoop = false;
float    prevLidarDist = 0.0f;
bool     edgeSeen      = false;

void stopRobot() { robot.stop_all(); }

bool turnDone()    { return millis() - turnStartMs >= TURN_MS; }
bool reverseDone() { return millis() - turnStartMs >= REVERSE_MS; }

bool newRfid() {
    if (!rfidThisLoop) return false;
    if (millis() - lastRfidMs < RFID_COOLDOWN_MS) return false;
    lastRfidMs = millis();
    return true;
}

bool rfidRecent() {
    return lastRfidMs != 0 && (millis() - lastRfidMs) < RFID_WINDOW_MS;
}

bool lidarFresh(LidarSensor& l) {
    return l.isValid() && (millis() - l.getLastUpdateMs()) <= LIDAR_FRESH_MS;
}

bool frontBlocked() {
    return lidarFresh(lidarFront)
        && lidarFront.getDistanceCM() > 0
        && lidarFront.getDistanceCM() <= OBSTACLE_CM;
}

// Returns true once when lidarLeft jumps from <=NEAR to >=FAR (obstacle edge passed)
bool checkEdge() {
    if (!lidarFresh(lidarLeft)) return false;
    float cur = (float)lidarLeft.getDistanceCM();
    bool edge = prevLidarDist > 0.0f
             && prevLidarDist <= EDGE_NEAR_CM
             && cur >= EDGE_FAR_CM;
    prevLidarDist = cur;
    return edge;
}

void resetEdge() { prevLidarDist = 0.0f; edgeSeen = false; }

void startTurn(Step next) {
    stopRobot();
    step        = next;
    turnStartMs = millis();
}

void startSeg(Step next) {
    step = next;
    lineFollower.reset();
}

// --- state handlers ---

void runForward() {
    lineFollower.update(DRIVE_SPEED);
    if (newRfid()) {
        Serial.println("[obs] rfid in forward");
        lastRfidMs = millis();
    }
    if (frontBlocked() && rfidRecent()) {
        Serial.println("[obs] obstacle + rfid -> reverse");
        resetEdge();
        startTurn(Step::Reverse);
    }
}

void runReverse() {
    robot.backward(REVERSE_SPEED);
    if (reverseDone()) {
        Serial.println("[obs] reverse done -> turn left");
        startTurn(Step::TurnLeft);
    }
}

void runTurnLeft() {
    robot.rotate_left(TURN_SPEED);
    if (turnDone()) {
        Serial.println("[obs] turn left done -> bypass line follow");
        resetEdge();
        lastRfidMs = 0;
        startSeg(Step::BypassLineFollow);
    }
}

void runBypassLineFollow() {
    lineFollower.update(DRIVE_SPEED);

    if (newRfid()) {
        Serial.println("[obs] bypass rfid detected");
        lastRfidMs = millis();
    }
    if (checkEdge()) {
        edgeSeen = true;
        Serial.println("[obs] edge seen during bypass");
    }
    if (edgeSeen && rfidRecent()) {
        Serial.println("[obs] edge + rfid -> turn right 1");
        resetEdge();
        lastRfidMs = 0;
        startTurn(Step::BypassTurnRight1);
    }
}

void runBypassTurnRight1() {
    robot.rotate_right(TURN_SPEED);
    if (turnDone()) {
        Serial.println("[obs] turn right 1 done -> bypass forward");
        resetEdge();
        startSeg(Step::BypassForward);
    }
}

void runBypassForward() {
    lineFollower.update(DRIVE_SPEED);
    if (checkEdge()) {
        Serial.println("[obs] second edge -> turn right 2");
        resetEdge();
        startTurn(Step::BypassTurnRight2);
    }
}

void runBypassTurnRight2() {
    robot.rotate_right(TURN_SPEED);
    if (turnDone()) {
        Serial.println("[obs] turn right 2 done -> find line");
        startSeg(Step::FindLine);
    }
}

void runFindLine() {
    // Let LineFollower search — it rotates toward last known line side.
    // Only advance to Resume once the line is actually detected.
    lineFollower.update(DRIVE_SPEED);
    if (lineFollower.hasLine()) {
        Serial.println("[obs] line found -> resume");
        step = Step::Resume;
    }
}

void runResume() {
    lineFollower.update(DRIVE_SPEED);
}

void updateControl() {
    if (!armed || !motorReady || !encReady) { stopRobot(); return; }
    switch (step) {
        case Step::Forward:          runForward();          break;
        case Step::Reverse:          runReverse();          break;
        case Step::TurnLeft:         runTurnLeft();         break;
        case Step::BypassLineFollow: runBypassLineFollow(); break;
        case Step::BypassTurnRight1: runBypassTurnRight1(); break;
        case Step::BypassForward:    runBypassForward();    break;
        case Step::BypassTurnRight2: runBypassTurnRight2(); break;
        case Step::FindLine:         runFindLine();         break;
        case Step::Resume:           runResume();           break;
    }
}

const char* stepName() {
    switch (step) {
        case Step::Forward:          return "FWD";
        case Step::Reverse:          return "REV";
        case Step::TurnLeft:         return "TL";
        case Step::BypassLineFollow: return "BYP_LINE";
        case Step::BypassTurnRight1: return "BYP_TR1";
        case Step::BypassForward:    return "BYP_FWD";
        case Step::BypassTurnRight2: return "BYP_TR2";
        case Step::FindLine:         return "FIND_LINE";
        case Step::Resume:           return "RESUME";
        default:                     return "?";
    }
}

void printDebug() {
    if (millis() - lastDebugMs < DEBUG_INTERVAL_MS) return;
    lastDebugMs = millis();
    Serial.print("step="); Serial.print(stepName());
    Serial.print("  F=");
    Serial.print(lidarFresh(lidarFront) ? lidarFront.getDistanceCM() : -1);
    Serial.print("cm  L=");
    Serial.print(lidarFresh(lidarLeft) ? lidarLeft.getDistanceCM() : -1);
    Serial.print("cm  rfid="); Serial.print(rfidRecent() ? "Y" : "-");
    Serial.print("  edge="); Serial.println(edgeSeen ? "Y" : "-");
}

void pollSerial() {
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == 'g' || c == 'G') {
            armed = true;
            step  = Step::Forward;
            lastRfidMs = 0;
            resetEdge();
            lineFollower.reset();
            Serial.println("Armed.");
        } else if (c == 'x' || c == 'X') {
            armed = false;
            stopRobot();
            Serial.println("Stopped.");
        }
    }
}

} // namespace

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}

    lineSensors.begin();
    lidarFront.begin();
    lidarLeft.begin();
    rfid.begin();

    motorReady = robot.begin();
    if (motorReady) {
        robot.set_max_speed(MOTOR_MAX_SPEED);
        robot.clear_status_flags();
        robot.stop_all();
        encReady = robot.begin_encoder_speed_control();
        lineFollower.reset();
    }

    Serial.print("motor="); Serial.print(motorReady ? "OK" : "FAILED");
    Serial.print("  encoder="); Serial.println(encReady ? "OK" : "FAILED");
    Serial.println("G = start   X = stop");
}

void loop() {
    rfidThisLoop = rfid.update();
    lidarFront.update();
    lidarLeft.update();

    pollSerial();
    updateControl();
    robot.update_encoder_speed_control();
    printDebug();
}
