#include <Arduino.h>

#include "IRConfig.h"
#include "IRSensor.h"
#include "MotorConfig.h"
#include "MotoronDrive.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t SERIAL_WAIT_MS = 3000;
constexpr uint32_t CONTROL_INTERVAL_MS = 30;
constexpr uint32_t DEBUG_INTERVAL_MS = 250;

constexpr int16_t LINE_SPEED = 220;
constexpr int16_t SEARCH_TURN_SPEED = 180;
constexpr int16_t MAX_TURN_SPEED = 650;

constexpr int16_t SENSOR_STEP = 1000;
constexpr uint16_t LINE_MIN_CONTRAST = 50;
constexpr float LINE_TURN_KP = 0.08f;
constexpr float LINE_TURN_KD = 0.003f;
constexpr bool START_ENABLED = false;

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
IRSensor lineSensors(IR_PINS, IR_SENSOR_COUNT);

bool motorReady = false;
bool driveEnabled = START_ENABLED;
bool haveLastLineError = false;
int16_t lastLineError = 0;
int16_t lastTurn = 0;
uint32_t lastControlAt = 0;
uint32_t lastLineSampleAt = 0;
uint32_t lastDebugAt = 0;

void waitForSerial(uint32_t timeoutMs)
{
    const uint32_t startMs = millis();

    while (!Serial && millis() - startMs < timeoutMs)
    {
        delay(10);
    }
}

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

void stopRobot()
{
    robot.stop_all();
}

void followLine()
{
    const uint32_t now = millis();

    if (now - lastControlAt < CONTROL_INTERVAL_MS)
    {
        return;
    }

    lastControlAt = now;

    if (!driveEnabled || !motorReady)
    {
        stopRobot();
        return;
    }

    int16_t error = 0;

    if (readLineError(error))
    {
        float dtSeconds = static_cast<float>(now - lastLineSampleAt) / 1000.0f;
        if (!haveLastLineError || lastLineSampleAt == 0 || dtSeconds <= 0.0f)
        {
            dtSeconds = 0.0f;
        }

        const float derivative = dtSeconds > 0.0f
            ? (static_cast<float>(error) - lastLineError) / dtSeconds
            : 0.0f;

        const int16_t turn = clampTurn(
            -(error * LINE_TURN_KP + derivative * LINE_TURN_KD)
        );

        robot.drive(LINE_SPEED, 0, turn);
        lastLineError = error;
        lastTurn = turn;
        lastLineSampleAt = now;
        haveLastLineError = true;
        return;
    }

    haveLastLineError = false;

    if (lastTurn < 0)
    {
        lastTurn = -SEARCH_TURN_SPEED;
    }
    else
    {
        lastTurn = SEARCH_TURN_SPEED;
    }

    robot.drive(0, 0, lastTurn);
}

void handleSerial()
{
    if (!Serial.available())
    {
        return;
    }

    const char command = Serial.read();

    switch (command)
    {
    case 'g':
    case 'G':
        driveEnabled = true;
        Serial.println("Line follow enabled.");
        break;

    case 's':
    case 'S':
    case ' ':
        driveEnabled = false;
        stopRobot();
        Serial.println("Line follow stopped.");
        break;

    default:
        break;
    }
}

void printDebug()
{
    if (millis() - lastDebugAt < DEBUG_INTERVAL_MS)
    {
        return;
    }

    lastDebugAt = millis();

    Serial.print("IR:");

    for (uint8_t i = 0; i < lineSensors.getCount(); i++)
    {
        Serial.print(' ');
        Serial.print(lineSensors.getValue(i));
    }

    Serial.print(" | enabled=");
    Serial.print(driveEnabled ? "yes" : "no");
    Serial.print(" | motor=");
    Serial.print(motorReady ? "OK" : "FAIL");
    Serial.print(" | err=");
    Serial.print(lastLineError);
    Serial.print(" | turn=");
    Serial.println(lastTurn);
}
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    waitForSerial(SERIAL_WAIT_MS);

    lineSensors.begin();

    motorReady = robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    stopRobot();

    Serial.println();
    Serial.println("Motor + IR line follow ready.");
    Serial.print("Motoron init: ");
    Serial.println(motorReady ? "OK" : "check Wire1 wiring or I2C address");
    Serial.println("Commands: g=start, s/space=stop");
    robot.print_status(Serial);
}

void loop()
{
    handleSerial();
    followLine();
    printDebug();
}
