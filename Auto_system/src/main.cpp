#include <Arduino.h>

#include "IRConfig.h"
#include "IRSensor.h"
#include "MotorConfig.h"
#include "MotoronDrive.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t DEBUG_INTERVAL_MS = 250;

constexpr int16_t LINE_SPEED = 200;
constexpr int16_t SEARCH_TURN_SPEED = 180;
constexpr int16_t MAX_TURN_SPEED = 800;

constexpr int16_t SENSOR_STEP = 1000;
constexpr uint16_t LINE_MIN_CONTRAST = 50;
constexpr float TURN_KP = 0.08f;

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
IRSensor lineSensors(IR_PINS, IR_SENSOR_COUNT);

int16_t lastTurn = 0;
uint32_t lastDebugAt = 0;

int16_t clampTurn(float turn)
{
    if (turn > MAX_TURN_SPEED)
    {
        return MAX_TURN_SPEED;
    }

    if (turn < -MAX_TURN_SPEED)
    {
        return -MAX_TURN_SPEED;
    }

    return static_cast<int16_t>(turn);
}

bool readLineError(int16_t& error)
{
    lineSensors.update();

    uint16_t minValue = IR_READ_TIMEOUT_US;
    uint16_t maxValue = 0;

    for (uint8_t i = 0; i < lineSensors.getCount(); i++)
    {
        const uint16_t value = lineSensors.getValue(i);

        if (value < minValue)
        {
            minValue = value;
        }

        if (value > maxValue)
        {
            maxValue = value;
        }
    }

    if (maxValue - minValue < LINE_MIN_CONTRAST)
    {
        return false;
    }

    int32_t weightedSum = 0;
    uint32_t signalSum = 0;
    const int16_t center = static_cast<int16_t>(lineSensors.getCount() - 1) / 2;

    for (uint8_t i = 0; i < lineSensors.getCount(); i++)
    {
        const uint16_t signal = lineSensors.getValue(i) - minValue;
        const int16_t position =
            (static_cast<int16_t>(i) - center) * SENSOR_STEP;

        weightedSum += static_cast<int32_t>(signal) * position;
        signalSum += signal;
    }

    if (signalSum == 0)
    {
        return false;
    }

    error = static_cast<int16_t>(weightedSum / static_cast<int32_t>(signalSum));
    return true;
}

void followLine()
{
    int16_t error = 0;

    if (readLineError(error))
    {
        const int16_t turn = clampTurn(error * TURN_KP);

        robot.drive(LINE_SPEED, 0, turn);
        lastTurn = turn;
        return;
    }

    if (lastTurn < 0)
    {
        robot.rotate_left(SEARCH_TURN_SPEED);
    }
    else
    {
        robot.rotate_right(SEARCH_TURN_SPEED);
    }
}

void printDebug()
{
    Serial.print("IR:");

    for (uint8_t i = 0; i < lineSensors.getCount(); i++)
    {
        Serial.print(' ');
        Serial.print(lineSensors.getValue(i));
    }

    Serial.print(" | turn=");
    Serial.println(lastTurn);
}
}

void setup()
{
    Serial.begin(SERIAL_BAUD);

    lineSensors.begin();

    const bool motoronReady = robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    robot.stop();

    Serial.println("Line following started.");
    Serial.print("Motoron init: ");
    Serial.println(motoronReady ? "OK" : "check wiring or I2C address");
    robot.print_status(Serial);
}

void loop()
{
    followLine();

    if (millis() - lastDebugAt >= DEBUG_INTERVAL_MS)
    {
        printDebug();
        lastDebugAt = millis();
    }
}
