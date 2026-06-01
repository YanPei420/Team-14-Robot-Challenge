#include <Arduino.h>
#include "LidarSensor.h"
#include "LidarConfig.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"
#include "RFIDHandler.h"

namespace {

// --- tuning ---
constexpr uint32_t CONTROL_INTERVAL_MS  = 40;
constexpr uint32_t DEBUG_INTERVAL_MS    = 250;
constexpr uint32_t LIDAR_FRESH_MS       = 300;
constexpr uint32_t TURN_90_MS           = 700;
constexpr uint32_t RFID_WINDOW_MS       = 2000;
constexpr uint32_t RFID_IGNORE_MS       = 900;

constexpr int16_t  FORWARD_SPEED        = 240;
constexpr int16_t  SEARCH_TURN_SPEED    = 170;
constexpr int16_t  TURN_SPEED           = 260;
constexpr int16_t  MAX_CORRECTION       = 260;

constexpr float    WALL_KP              = 45.0f;
constexpr float    TARGET_LEFT_CM       = 6.0f;
constexpr float    TARGET_RIGHT_CM      = 10.0f;
constexpr float    WALL_DEADBAND_CM     = 1.5f;
constexpr float    WALL_VISIBLE_MAX_CM  = 55.0f;

constexpr float    FRONT_OBSTACLE_CM    = 10.0f;

constexpr float    RIGHT_EDGE_NEAR_CM   = 18.0f;
constexpr float    RIGHT_EDGE_FAR_CM    = 35.0f;
constexpr float    RIGHT_EDGE_JUMP_CM   = 15.0f;

// --- hardware ---
LidarSensor  lidarLeft (LIDAR_SERIAL_LEFT);
LidarSensor  lidarRight(LIDAR_SERIAL_RIGHT);
LidarSensor  lidarFront(LIDAR_SERIAL_FRONT);
MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
RFIDHandler  rfid;

// --- states ---
enum class State : uint8_t {
    Disarmed,
    MotorError,

    FollowLeft,

    TurnLeft1,
    Forward1,
    TurnRight1,

    FollowRight,

    TurnRight2,
    Forward2,
    TurnLeft2,
};

State    state         = State::Disarmed;
bool     armed         = false;
bool     motorReady    = false;

uint32_t lastControlMs = 0;
uint32_t lastDebugMs   = 0;
uint32_t turnStartMs   = 0;

uint32_t lastRfidMs        = 0;
uint32_t ignoreRfidUntilMs = 0;

float    prevRightDist = 0.0f;
bool     edgeSeen      = false;
uint8_t  bypassRfidCount = 0;

// --- helpers ---
bool lidarFresh(LidarSensor& l) {
    return l.isValid() && (millis() - l.getLastUpdateMs()) <= LIDAR_FRESH_MS;
}

bool leftWallVisible() {
    return lidarFresh(lidarLeft)
        && lidarLeft.getDistanceCM() > 0
        && lidarLeft.getDistanceCM() <= WALL_VISIBLE_MAX_CM;
}

bool frontBlocked() {
    return lidarFresh(lidarFront)
        && lidarFront.getDistanceCM() > 0
        && lidarFront.getDistanceCM() <= FRONT_OBSTACLE_CM;
}

bool rfidUsable() {
    return lastRfidMs != 0
        && millis() >= ignoreRfidUntilMs
        && (millis() - lastRfidMs) < RFID_WINDOW_MS;
}

void consumeRfid() {
    lastRfidMs = 0;
    ignoreRfidUntilMs = millis() + RFID_IGNORE_MS;
}

bool checkRightEdge() {
    if (!lidarFresh(lidarRight)) return false;

    float cur = (float)lidarRight.getDistanceCM();

    bool edge =
        prevRightDist > 0.0f &&
        prevRightDist <= RIGHT_EDGE_NEAR_CM &&
        cur >= RIGHT_EDGE_FAR_CM &&
        (cur - prevRightDist) >= RIGHT_EDGE_JUMP_CM;

    prevRightDist = cur;
    return edge;
}

int16_t clamp(float v) {
    if (v > MAX_CORRECTION) return MAX_CORRECTION;
    if (v < -MAX_CORRECTION) return -MAX_CORRECTION;
    return (int16_t)v;
}

void stopRobot() {
    robot.stop_all();
}

void startTurn(State next) {
    state = next;
    turnStartMs = millis();
}

bool turnDone() {
    return millis() - turnStartMs >= TURN_90_MS;
}

// --- state handlers ---
void runFollowLeft() {
    if (frontBlocked() && rfidUsable()) {
        Serial.println("[bypass] obstacle + RFID P1 -> TurnLeft1");
        consumeRfid();

        bypassRfidCount = 0;
        edgeSeen = false;
        prevRightDist = 0.0f;

        startTurn(State::TurnLeft1);
        return;
    }

    if (!leftWallVisible()) {
        robot.rotate_left(SEARCH_TURN_SPEED);
        return;
    }

    float err = TARGET_LEFT_CM - (float)lidarLeft.getDistanceCM();
    int16_t w = (fabsf(err) > WALL_DEADBAND_CM)
        ? clamp(err * WALL_KP)
        : 0;

    robot.drive(FORWARD_SPEED, 0, w);
}

void runTurnLeft1() {
    robot.rotate_left(TURN_SPEED);

    if (turnDone()) {
        Serial.println("[bypass] TurnLeft1 done -> Forward1");
        state = State::Forward1;
    }
}

void runForward1() {
    if (rfidUsable()) {
        Serial.println("[bypass] RFID P2 -> TurnRight1");
        consumeRfid();
        startTurn(State::TurnRight1);
        return;
    }

    robot.drive(FORWARD_SPEED, 0, 0);
}

void runTurnRight1() {
    robot.rotate_right(TURN_SPEED);

    if (turnDone()) {
        Serial.println("[bypass] TurnRight1 done -> FollowRight");

        prevRightDist = 0.0f;
        edgeSeen = false;
        bypassRfidCount = 0;

        state = State::FollowRight;
    }
}

void runFollowRight() {
    if (checkRightEdge()) {
        edgeSeen = true;
        Serial.println("[bypass] right edge detected");
    }

    if (rfidUsable()) {
        bypassRfidCount++;
        Serial.print("[bypass] RFID during FollowRight, count=");
        Serial.println(bypassRfidCount);
        consumeRfid();
    }

    // Trigger when BOTH conditions are met, regardless of which arrived first
    if (edgeSeen && bypassRfidCount >= 2) {
        Serial.println("[bypass] edge + second RFID -> TurnRight2");
        edgeSeen = false;
        prevRightDist = 0.0f;
        startTurn(State::TurnRight2);
        return;
    }

    float cur = lidarFresh(lidarRight)
        ? (float)lidarRight.getDistanceCM()
        : TARGET_RIGHT_CM;

    float err = TARGET_RIGHT_CM - cur;

    int16_t w = (fabsf(err) > WALL_DEADBAND_CM)
        ? clamp(-err * WALL_KP)
        : 0;

    robot.drive(FORWARD_SPEED, 0, w);
}

void runTurnRight2() {
    robot.rotate_right(TURN_SPEED);

    if (turnDone()) {
        Serial.println("[bypass] TurnRight2 done -> Forward2");
        state = State::Forward2;
    }
}

void runForward2() {
    if (rfidUsable()) {
        Serial.println("[bypass] final RFID -> TurnLeft2");
        consumeRfid();
        startTurn(State::TurnLeft2);
        return;
    }

    robot.drive(FORWARD_SPEED, 0, 0);
}

void runTurnLeft2() {
    robot.rotate_left(TURN_SPEED);

    if (turnDone()) {
        Serial.println("[bypass] TurnLeft2 done -> resume FollowLeft");

        bypassRfidCount = 0;
        edgeSeen = false;
        prevRightDist = 0.0f;

        state = State::FollowLeft;
    }
}

// --- main control ---
void updateControl() {
    if (millis() - lastControlMs < CONTROL_INTERVAL_MS) return;
    lastControlMs = millis();

    if (!motorReady) {
        state = State::MotorError;
        stopRobot();
        return;
    }

    if (!armed) {
        state = State::Disarmed;
        stopRobot();
        return;
    }

    switch (state) {
        case State::FollowLeft:
            runFollowLeft();
            break;

        case State::TurnLeft1:
            runTurnLeft1();
            break;

        case State::Forward1:
            runForward1();
            break;

        case State::TurnRight1:
            runTurnRight1();
            break;

        case State::FollowRight:
            runFollowRight();
            break;

        case State::TurnRight2:
            runTurnRight2();
            break;

        case State::Forward2:
            runForward2();
            break;

        case State::TurnLeft2:
            runTurnLeft2();
            break;

        default:
            stopRobot();
            break;
    }
}

// --- debug ---
const char* stateName(State s) {
    switch (s) {
        case State::Disarmed:    return "DISARMED";
        case State::MotorError:  return "MOTOR_ERR";
        case State::FollowLeft:  return "FOLLOW_L";
        case State::TurnLeft1:   return "TL1";
        case State::Forward1:    return "FWD1";
        case State::TurnRight1:  return "TR1";
        case State::FollowRight: return "FOLLOW_R";
        case State::TurnRight2:  return "TR2";
        case State::Forward2:    return "FWD2";
        case State::TurnLeft2:   return "TL2";
        default:                 return "?";
    }
}

void printDebug() {
    if (millis() - lastDebugMs < DEBUG_INTERVAL_MS) return;
    lastDebugMs = millis();

    Serial.print("state=");
    Serial.print(stateName(state));

    Serial.print("  L=");
    Serial.print(lidarFresh(lidarLeft) ? lidarLeft.getDistanceCM() : -1);

    Serial.print("  R=");
    Serial.print(lidarFresh(lidarRight) ? lidarRight.getDistanceCM() : -1);

    Serial.print("  F=");
    Serial.print(lidarFresh(lidarFront) ? lidarFront.getDistanceCM() : -1);

    Serial.print("  rfid=");
    Serial.print(rfidUsable() ? "YES" : "-");

    Serial.print("  edge=");
    Serial.print(edgeSeen ? "YES" : "-");

    Serial.print("  count=");
    Serial.println(bypassRfidCount);
}

// --- serial command ---
void pollSerial() {
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();

        if (c == 'g' || c == 'G') {
            armed = true;
            state = State::FollowLeft;

            lastRfidMs = 0;
            ignoreRfidUntilMs = millis() + RFID_IGNORE_MS;
            bypassRfidCount = 0;
            edgeSeen = false;
            prevRightDist = 0.0f;

            Serial.println("Armed - wall following started.");
        }

        else if (c == 'x' || c == 'X') {
            armed = false;
            stopRobot();
            state = State::Disarmed;
            Serial.println("Stopped.");
        }
    }
}

} // namespace

void setup() {
    Serial.begin(115200);

    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) { ; }

    lidarLeft.begin();
    lidarRight.begin();
    lidarFront.begin();
    rfid.begin();

    motorReady = robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    stopRobot();

    Serial.println("Wall-follow + obstacle bypass ready.");
    Serial.println("G = arm/start   X = stop");
    Serial.print("Motoron: ");
    Serial.println(motorReady ? "OK" : "FAILED");
}

void loop() {
    lidarLeft.update();
    lidarRight.update();
    lidarFront.update();

    if (rfid.update()) {
        lastRfidMs = millis();
    }

    pollSerial();
    updateControl();
    printDebug();
}