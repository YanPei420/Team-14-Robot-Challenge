#include <Arduino.h>
#include "LidarSensor.h"
#include "LidarConfig.h"
#include "MotoronDrive.h"
#include "MotorConfig.h"
#include "WallFollower.h"

LidarSensor  lidarLeft (LIDAR_SERIAL_LEFT);
LidarSensor  lidarRight(LIDAR_SERIAL_RIGHT);
MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

WallFollowerConfig wfConfig;
WallFollower wallFollower(robot, lidarLeft, lidarRight, wfConfig);

bool motorReady          = false;
bool encoderControlReady = false;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}

    wfConfig.targetWallDistanceCm = 6.0f;

    lidarLeft.begin();
    lidarRight.begin();

    motorReady = robot.begin();
    if (motorReady) {
        robot.set_max_speed(MOTOR_MAX_SPEED);
        robot.clear_status_flags();
        robot.stop_all();
        encoderControlReady = robot.begin_encoder_speed_control();
        wallFollower.reset();
    }

    Serial.print("motor=");
    Serial.print(motorReady ? "OK" : "FAILED");
    Serial.print("  encoder_control=");
    Serial.println(encoderControlReady ? "OK" : "FAILED");
}

void loop() {
    lidarLeft.update();
    lidarRight.update();

    if (!motorReady || !encoderControlReady) {
        robot.stop_all();
        return;
    }

    wallFollower.update(140);
    robot.update_encoder_speed_control();
}
