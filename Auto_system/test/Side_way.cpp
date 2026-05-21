#include <Arduino.h>
#include <Wire.h>

#include "MotorConfig.h"
#include "MotoronDrive.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr int16_t STRAFE_SPEED = 250;
constexpr uint32_t COMMAND_REFRESH_MS = 100;

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

uint32_t lastCommandAt = 0;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);

    while (!Serial)
    {
        ;
    }

    Wire1.begin();

    robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.stop();

    Serial.println("Robot strafe right test started.");
}

void loop()
{
    if (millis() - lastCommandAt < COMMAND_REFRESH_MS)
    {
        return;
    }

    robot.right(STRAFE_SPEED);
    lastCommandAt = millis();
}
