#include <Arduino.h>
#include "LidarSensor.h"
#include "LidarConfig.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"

// --- tuning (same for both up and down) ---
constexpr int16_t  FORWARD_SPEED     = 260;
constexpr int16_t  MAX_CORRECTION    = 200;
constexpr float    TARGET_CM         = 4.5f;
constexpr float    WALL_DEADBAND_CM  = 1.0f;
constexpr float    WALL_KP           = 10.0f;
constexpr float    WALL_KD           = 8.2f;
constexpr float    WALL_MAX_CM       = 50.0f;
constexpr uint32_t LIDAR_FRESH_MS    = 300;
constexpr uint32_t DEBUG_INTERVAL_MS = 200;

LidarSensor  lidarLeft (LIDAR_SERIAL_LEFT);   // Serial2 - up
LidarSensor  lidarRight(LIDAR_SERIAL_RIGHT);  // Serial3 - down
MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

enum class Mode { IDLE, UP, DOWN };

Mode     mode                = Mode::IDLE;
bool     motorReady          = false;
bool     encoderControlReady = false;
uint32_t lastDebugMs         = 0;
float    prevError           = 0.0f;
uint32_t prevControlMs       = 0;

bool lidarFresh(LidarSensor& l)
{
    return l.isValid() && (millis() - l.getLastUpdateMs()) <= LIDAR_FRESH_MS;
}

bool leftVisible()
{
    return lidarFresh(lidarLeft)
        && lidarLeft.getDistanceCM() > 0
        && lidarLeft.getDistanceCM() <= WALL_MAX_CM;
}

bool rightVisible()
{
    return lidarFresh(lidarRight)
        && lidarRight.getDistanceCM() > 0
        && lidarRight.getDistanceCM() <= WALL_MAX_CM;
}

int16_t clamp(float v)
{
    if (v >  MAX_CORRECTION) return  MAX_CORRECTION;
    if (v < -MAX_CORRECTION) return -MAX_CORRECTION;
    return (int16_t)v;
}

// PD control — sign is positive for left wall, negative for right wall
int16_t pdCorrection(float dist, float sign)
{
    uint32_t now   = millis();
    float    error = dist - TARGET_CM;

    float dt     = (prevControlMs != 0) ? (now - prevControlMs) / 1000.0f : 0.0f;
    float dError = (dt > 0.005f) ? (error - prevError) / dt : 0.0f;

    prevError     = error;
    prevControlMs = now;

    if (fabsf(error) <= WALL_DEADBAND_CM) return 0;
    return clamp(sign * (WALL_KP * error + WALL_KD * dError));
}

void wallFollowUp()
{
    if (!leftVisible())
    {
        robot.drive(FORWARD_SPEED, 0, 0);
        prevError = 0.0f; prevControlMs = 0;
        return;
    }
    robot.drive(FORWARD_SPEED, 0, pdCorrection(lidarLeft.getDistanceCM(), +1.0f));
}

void wallFollowDown()
{
    if (!rightVisible())
    {
        robot.drive(FORWARD_SPEED, 0, 0);
        prevError = 0.0f; prevControlMs = 0;
        return;
    }
    robot.drive(FORWARD_SPEED, 0, pdCorrection(lidarRight.getDistanceCM(), -1.0f));
}

void printDebug()
{
    if (millis() - lastDebugMs < DEBUG_INTERVAL_MS) return;
    lastDebugMs = millis();

    if (mode == Mode::UP) {
        Serial.print("UP  L=");
        Serial.print(lidarFresh(lidarLeft)  ? lidarLeft.getDistanceCM()  : -1);
        Serial.print("cm  err=");
        Serial.println(lidarFresh(lidarLeft) ? lidarLeft.getDistanceCM() - TARGET_CM : 0.0f, 1);
    } else if (mode == Mode::DOWN) {
        Serial.print("DOWN  R=");
        Serial.print(lidarFresh(lidarRight) ? lidarRight.getDistanceCM() : -1);
        Serial.print("cm  err=");
        Serial.println(lidarFresh(lidarRight) ? lidarRight.getDistanceCM() - TARGET_CM : 0.0f, 1);
    } else {
        Serial.println("IDLE  U=up  D=down  X=stop");
    }
}

void pollSerial()
{
    while (Serial.available() > 0)
    {
        const char c = (char)Serial.read();
        if (c == 'u' || c == 'U') {
            mode = Mode::UP;
            prevError = 0.0f; prevControlMs = 0;
            Serial.println("UP - left LIDAR (Serial2)");
        } else if (c == 'd' || c == 'D') {
            mode = Mode::DOWN;
            prevError = 0.0f; prevControlMs = 0;
            Serial.println("DOWN - right LIDAR (Serial3)");
        } else if (c == 'x' || c == 'X') {
            mode = Mode::IDLE;
            robot.stop_all();
            Serial.println("Stopped.");
        }
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}

    lidarLeft.begin();
    lidarRight.begin();

    motorReady = robot.begin();
    if (motorReady)
    {
        robot.set_max_speed(MOTOR_MAX_SPEED);
        robot.clear_status_flags();
        robot.stop_all();
        encoderControlReady = robot.begin_encoder_speed_control();
    }

    Serial.print("motor=");
    Serial.print(motorReady ? "OK" : "FAILED");
    Serial.print("  encoder_control=");
    Serial.println(encoderControlReady ? "OK" : "FAILED");
    Serial.println("U = up (left LIDAR)   D = down (right LIDAR)   X = stop");
}

void loop()
{
    lidarLeft.update();
    lidarRight.update();

    pollSerial();

    if (!motorReady || !encoderControlReady || mode == Mode::IDLE)
    {
        robot.stop_all();
    }
    else if (mode == Mode::UP)
    {
        wallFollowUp();
    }
    else
    {
        wallFollowDown();
    }

    robot.update_encoder_speed_control();
    printDebug();
}
