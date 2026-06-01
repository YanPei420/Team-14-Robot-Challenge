#include <Arduino.h>
#include "LidarSensor.h"
#include "LidarConfig.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"

// --- tuning ---
constexpr int16_t  FORWARD_SPEED     = 260;
constexpr int16_t  MAX_CORRECTION    = 200;
constexpr float    TARGET_LEFT_CM    = 4.5f;
constexpr float    WALL_DEADBAND_CM  = 1.0f;
constexpr float    WALL_KP           = 10.0f;  // increase if too sluggish
constexpr float    WALL_KD           = 8.2f;   // increase if oscillating
constexpr float    WALL_MAX_CM       = 50.0f;
constexpr uint32_t LIDAR_FRESH_MS    = 300;
constexpr uint32_t DEBUG_INTERVAL_MS = 200;

LidarSensor  lidarLeft(LIDAR_SERIAL_LEFT);
MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

bool     armed               = false;
bool     motorReady          = false;
bool     encoderControlReady = false;
uint32_t lastDebugMs         = 0;

float    prevError     = 0.0f;
uint32_t prevControlMs = 0;

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

int16_t clamp(float v)
{
    if (v >  MAX_CORRECTION) return  MAX_CORRECTION;
    if (v < -MAX_CORRECTION) return -MAX_CORRECTION;
    return (int16_t)v;
}

void wallFollow()
{
    uint32_t now = millis();

    if (!leftVisible())
    {
        robot.drive(FORWARD_SPEED, 0, 0);
        prevError     = 0.0f;
        prevControlMs = 0;
        return;
    }

    float error = (float)lidarLeft.getDistanceCM() - TARGET_LEFT_CM;

    float dt     = (prevControlMs != 0) ? (now - prevControlMs) / 1000.0f : 0.0f;
    float dError = (dt > 0.005f) ? (error - prevError) / dt : 0.0f;

    prevError     = error;
    prevControlMs = now;

    int16_t w = 0;
    if (fabsf(error) > WALL_DEADBAND_CM)
        w = clamp(WALL_KP * error + WALL_KD * dError);

    robot.drive(FORWARD_SPEED, 0, w);
}

void printDebug()
{
    if (millis() - lastDebugMs < DEBUG_INTERVAL_MS) return;
    lastDebugMs = millis();

    Serial.print("L=");
    Serial.print(lidarFresh(lidarLeft) ? lidarLeft.getDistanceCM() : -1);
    Serial.print("cm  err=");
    Serial.print(lidarFresh(lidarLeft) ? lidarLeft.getDistanceCM() - TARGET_LEFT_CM : 0.0f, 1);
    Serial.print("  ");
    Serial.println(armed ? "RUNNING" : "DISARMED");
}

void pollSerial()
{
    while (Serial.available() > 0)
    {
        const char c = (char)Serial.read();
        if (c == 'g' || c == 'G') {
            armed         = true;
            prevError     = 0.0f;
            prevControlMs = 0;
            Serial.println("Armed - wall following started.");
        } else if (c == 'x' || c == 'X') {
            armed = false;
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
    Serial.println("G = start   X = stop");
}

void loop()
{
    lidarLeft.update();

    pollSerial();

    if (!motorReady || !encoderControlReady || !armed)
    {
        robot.stop_all();
    }
    else
    {
        wallFollow();
    }

    robot.update_encoder_speed_control();
    printDebug();
}
