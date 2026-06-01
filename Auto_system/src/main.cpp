#include <Arduino.h>

#include "LineFollower.h"

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
IRSensor ir(IR_PINS, IR_SENSOR_COUNT);
LineFollower lineFollower(robot, ir);

bool motorReady = false;
bool encoderControlReady = false;

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}

    ir.begin();

    motorReady = robot.begin();
    if (motorReady)
    {
        robot.set_max_speed(MOTOR_MAX_SPEED);
        robot.clear_status_flags();
        robot.stop_all();
        encoderControlReady = robot.begin_encoder_speed_control();
        lineFollower.reset();
    }

    lineFollower.setLogOutput(&Serial);

    Serial.print("motor=");
    Serial.print(motorReady ? "OK" : "FAILED");
    Serial.print(" encoder_control=");
    Serial.println(encoderControlReady ? "OK" : "FAILED");
}

void loop()
{
    if (!motorReady || !encoderControlReady)
    {
        robot.stop_all();
        return;
    }

    lineFollower.update();
}
