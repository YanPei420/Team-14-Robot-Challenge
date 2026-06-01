#include <Arduino.h>

#include "LidarSensor.h"
#include "MotorConfig.h"
#include "MotoronDrive.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t CONTROL_INTERVAL_MS = 40;
constexpr uint32_t DEBUG_INTERVAL_MS = 250;
constexpr uint32_t LIDAR_FRESH_MS = 300;

constexpr int16_t FORWARD_SPEED = 240;
constexpr int16_t SEARCH_TURN_SPEED = 170;
constexpr int16_t AVOID_TURN_SPEED = 260;
constexpr int16_t MAX_CORRECTION_SPEED = 260;

constexpr float TARGET_LEFT_DISTANCE_CM = 10.0f;
constexpr float WALL_DEADBAND_CM = 1.5f;
constexpr float WALL_VISIBLE_MAX_CM = 55.0f;
constexpr float FRONT_OBSTACLE_CM = 22.0f;
constexpr float WALL_TURN_KP = 45.0f;

LidarSensor lidarLeft(LIDAR_SERIAL_1);
LidarSensor lidarRight(LIDAR_SERIAL_2);
LidarSensor lidarFront(LIDAR_SERIAL_3);
MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

enum class WallState : uint8_t
{
    Disarmed,
    MotorError,
    AvoidFront,
    SearchWall,
    FollowWall
};

WallState state = WallState::Disarmed;
bool armed = false;
bool motorReady = false;
uint32_t lastControlMs = 0;
uint32_t lastDebugMs = 0;

const char* stateName(WallState value)
{
    switch (value)
    {
    case WallState::Disarmed: return "DISARMED";
    case WallState::MotorError: return "MOTOR_ERROR";
    case WallState::AvoidFront: return "AVOID_FRONT";
    case WallState::SearchWall: return "SEARCH_WALL";
    case WallState::FollowWall: return "FOLLOW_WALL";
    default: return "UNKNOWN";
    }
}

bool lidarFresh(LidarSensor& lidar)
{
    return lidar.isValid() && millis() - lidar.getLastUpdateMs() <= LIDAR_FRESH_MS;
}

bool frontObstacleClose()
{
    return lidarFresh(lidarLeft)
        && lidarLeft.getDistanceCM() > 0
        && lidarLeft.getDistanceCM() <= WALL_VISIBLE_MAX_CM;
}

bool frontBlocked()
{
    return lidarFresh(lidarFront)
        && lidarFront.getDistanceCM() > 0
        && lidarFront.getDistanceCM() <= FRONT_OBSTACLE_CM;
}

int16_t clampCorrection(float value)
{
    if (value > MAX_CORRECTION_SPEED)
    {
        return MAX_CORRECTION_SPEED;
    }

    if (value < -MAX_CORRECTION_SPEED)
    {
        return -MAX_CORRECTION_SPEED;
    }

    return static_cast<int16_t>(value);
}

void stopRobot()
{
    robot.stop_all();
}

void printHelp()
{
    Serial.println("Wall-follow test commands:");
    Serial.println("  G - arm and start following the left wall");
    Serial.println("  X - stop and disarm");
    Serial.println("Keep the robot lifted or restrained for the first run.");
}

void pollSerial()
{
    while (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());

        if (command == 'g' || command == 'G')
        {
            armed = true;
            Serial.println("Wall-follow armed.");
        }
        else if (command == 'x' || command == 'X')
        {
            armed = false;
            stopRobot();
            Serial.println("Wall-follow stopped.");
        }
        else if (command == '?' || command == 'h' || command == 'H')
        {
            printHelp();
        }
    }
}

void updateLidars()
{
    lidarLeft.update();
    lidarRight.update();
    lidarFront.update();
}

void updateWallFollow()
{
    const uint32_t now = millis();

    if (now - lastControlMs < CONTROL_INTERVAL_MS)
    {
        return;
    }
    lastControlMs = now;

    if (!motorReady)
    {
        state = WallState::MotorError;
        stopRobot();
        return;
    }

    if (!armed)
    {
        state = WallState::Disarmed;
        stopRobot();
        return;
    }

    if (frontBlocked())
    {
        state = WallState::AvoidFront;
        robot.rotate_right(AVOID_TURN_SPEED);
        return;
    }

    if (!leftWallVisible())
    {
        state = WallState::SearchWall;
        robot.rotate_left(SEARCH_TURN_SPEED);
        return;
    }

    state = WallState::FollowWall;

    const float distance = static_cast<float>(lidarLeft.getDistanceCM());
    const float error = TARGET_LEFT_DISTANCE_CM - distance;
    int16_t turn = 0;

    if (abs(error) > WALL_DEADBAND_CM)
    {
        turn = clampCorrection(error * WALL_TURN_KP);
    }

    robot.drive(FORWARD_SPEED, 0, turn);
}

void printReading(const char* label, LidarSensor& lidar)
{
    Serial.print(label);
    Serial.print('=');

    if (!lidarFresh(lidar))
    {
        Serial.print("stale");
        return;
    }

    Serial.print(lidar.getDistanceCM());
    Serial.print("cm");
}

void printDebug()
{
    const uint32_t now = millis();

    if (now - lastDebugMs < DEBUG_INTERVAL_MS)
    {
        return;
    }
    lastDebugMs = now;

    int16_t frontLeft = 0;
    int16_t frontRight = 0;
    int16_t rearLeft = 0;
    int16_t rearRight = 0;
    robot.get_wheel_speeds(frontLeft, frontRight, rearLeft, rearRight);

    Serial.print("state=");
    Serial.print(stateName(state));
    Serial.print(" armed=");
    Serial.print(armed ? 1 : 0);
    Serial.print(" ");
    printReading("L", lidarLeft);
    Serial.print(" ");
    printReading("R", lidarRight);
    Serial.print(" ");
    printReading("F", lidarFront);
    Serial.print(" wheels=");
    Serial.print(frontLeft);
    Serial.print(',');
    Serial.print(frontRight);
    Serial.print(',');
    Serial.print(rearLeft);
    Serial.print(',');
    Serial.println(rearRight);
}
} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    const uint32_t serialStartMs = millis();

    while (!Serial && millis() - serialStartMs < 3000)
    {
        ;
    }

    lidarLeft.begin();
    lidarFront.begin();

    motorReady = robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    stopRobot();

    Serial.println("Wall-follow test ready.");
    Serial.print("Motoron init: ");
    Serial.println(motorReady ? "OK" : "FAILED");
    printHelp();
}

void loop()
{
    pollSerial();
    updateLidars();
    updateWallFollow();
    printDebug();
}
