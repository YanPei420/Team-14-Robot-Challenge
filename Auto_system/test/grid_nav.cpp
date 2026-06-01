// Task 3 - Solid Grid Navigation
// Route: forward 2 nodes -> turn right -> forward 1 node -> turn left -> forward 2 nodes -> stop
// Uses IR line following + RFID node counting (6 tags total)

#include <Arduino.h>
#include "IRConfig.h"
#include "IRSensor.h"
#include "LineFollower.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"
#include "RFIDHandler.h"

namespace {

// --- tuning ---
constexpr int16_t  DRIVE_SPEED       = 220;
constexpr int16_t  TURN_SPEED        = 260;
constexpr uint32_t TURN_MS           = 700;    // tune for 90 deg in-place turn
constexpr uint32_t RFID_COOLDOWN_MS  = 1500;   // prevent double-counting same tag
constexpr uint32_t DEBUG_INTERVAL_MS = 300;
// ---

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
IRSensor     lineSensors(IR_PINS, IR_SENSOR_COUNT);
LineFollower lineFollower(robot, lineSensors);
RFIDHandler  rfid;

// Steps in order: seg1(2 rfids) -> turn R -> seg2(1 rfid) -> turn L -> seg3(2 rfids) -> done
enum class Step : uint8_t {
    Seg1,
    TurnRight,
    Seg2,
    TurnLeft,
    Seg3,
    Done,
};

Step     step           = Step::Seg1;
bool     armed          = false;
bool     motorReady     = false;
uint8_t  rfidCount      = 0;        // rfids counted in current segment
uint32_t turnStartMs    = 0;
uint32_t lastRfidMs     = 0;        // timestamp of last accepted rfid read
uint32_t lastDebugMs    = 0;
bool     rfidThisLoop   = false;    // set by rfid.update() each loop

void stopRobot() { robot.stop_all(); }

bool turnDone() { return millis() - turnStartMs >= TURN_MS; }

// Accepts a new RFID read only if cooldown has elapsed (avoids double-counting)
bool newNode() {
    if (!rfidThisLoop) return false;
    if (millis() - lastRfidMs < RFID_COOLDOWN_MS) return false;
    lastRfidMs = millis();
    return true;
}

void startTurn(Step next) {
    stopRobot();
    step        = next;
    turnStartMs = millis();
}

void startSegment(Step next) {
    step      = next;
    rfidCount = 0;
    lineFollower.reset();
}

const char* stepName() {
    switch (step) {
        case Step::Seg1:      return "SEG1";
        case Step::TurnRight: return "TURN_R";
        case Step::Seg2:      return "SEG2";
        case Step::TurnLeft:  return "TURN_L";
        case Step::Seg3:      return "SEG3";
        case Step::Done:      return "DONE";
        default:              return "?";
    }
}

void runSeg1() {
    lineFollower.update(DRIVE_SPEED);
    if (newNode()) {
        rfidCount++;
        Serial.print("[nav] seg1 node "); Serial.println(rfidCount);
        if (rfidCount >= 2) {
            Serial.println("[nav] -> TurnRight");
            startTurn(Step::TurnRight);
        }
    }
}

void runTurnRight() {
    robot.rotate_right(TURN_SPEED);
    if (turnDone()) {
        Serial.println("[nav] -> Seg2");
        startSegment(Step::Seg2);
    }
}

void runSeg2() {
    lineFollower.update(DRIVE_SPEED);
    if (newNode()) {
        rfidCount++;
        Serial.print("[nav] seg2 node "); Serial.println(rfidCount);
        if (rfidCount >= 1) {
            Serial.println("[nav] -> TurnLeft");
            startTurn(Step::TurnLeft);
        }
    }
}

void runTurnLeft() {
    robot.rotate_left(TURN_SPEED);
    if (turnDone()) {
        Serial.println("[nav] -> Seg3");
        startSegment(Step::Seg3);
    }
}

void runSeg3() {
    lineFollower.update(DRIVE_SPEED);
    if (newNode()) {
        rfidCount++;
        Serial.print("[nav] seg3 node "); Serial.println(rfidCount);
        if (rfidCount >= 2) {
            stopRobot();
            step = Step::Done;
            Serial.println("[nav] DONE - route complete");
        }
    }
}

void updateControl() {
    if (!armed || !motorReady) { stopRobot(); return; }

    switch (step) {
        case Step::Seg1:      runSeg1();      break;
        case Step::TurnRight: runTurnRight(); break;
        case Step::Seg2:      runSeg2();      break;
        case Step::TurnLeft:  runTurnLeft();  break;
        case Step::Seg3:      runSeg3();      break;
        case Step::Done:      stopRobot();    break;
    }
}

void pollSerial() {
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == 'g' || c == 'G') {
            armed = true;
            startSegment(Step::Seg1);
            lastRfidMs = 0;
            Serial.println("Armed - grid nav started.");
        } else if (c == 'x' || c == 'X') {
            armed = false;
            stopRobot();
            Serial.println("Stopped.");
        }
    }
}

void printDebug() {
    if (millis() - lastDebugMs < DEBUG_INTERVAL_MS) return;
    lastDebugMs = millis();
    Serial.print("step="); Serial.print(stepName());
    Serial.print("  nodes="); Serial.print(rfidCount);
    Serial.print("  line="); Serial.println(lineFollower.hasLine() ? "yes" : "no");
}

} // namespace

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) { ; }

    lineSensors.begin();
    rfid.begin();

    motorReady = robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();

    if (motorReady) {
        robot.begin_encoder_speed_control();
    }

    stopRobot();

    Serial.println("Grid navigation test ready.");
    Serial.println("Route: fwd 2 -> right -> fwd 1 -> left -> fwd 2 -> stop");
    Serial.println("G = start   X = stop");
    Serial.print("Motoron: "); Serial.println(motorReady ? "OK" : "FAILED");
}

void loop() {
    rfidThisLoop = rfid.update();

    pollSerial();
    updateControl();
    robot.update_encoder_speed_control();
    printDebug();
}
