#include "system.h"

#include <Arduino.h>

#if defined(CORE_CM7)

#include "IRConfig.h"
#include "IRSensor.h"
#include "MotorConfig.h"
#include "MotoronDrive.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t CONTROL_INTERVAL_MS = 30;
constexpr uint32_t STATUS_INTERVAL_MS = 500;

constexpr int16_t LINE_SPEED = 220;
constexpr int16_t SEARCH_TURN_SPEED = 180;
constexpr int16_t MAX_TURN_SPEED = 650;
constexpr int16_t SENSOR_STEP = 1000;

constexpr uint16_t LINE_MIN_CONTRAST = 50;
constexpr float LINE_TURN_KP = 0.08f;
constexpr float LINE_TURN_KD = 0.003f;
constexpr float LINE_TURN_DIRECTION = -1.0f;

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
IRSensor lineSensors(IR_PINS, IR_SENSOR_COUNT);

bool motorReady = false;
bool haveLastLineError = false;

uint32_t lastControlMs = 0;
uint32_t lastLineMs = 0;
uint32_t lastStatusMs = 0;

int16_t lastLineError = 0;
int16_t lastTurn = SEARCH_TURN_SPEED;

void stopRobot()
{
    robot.stop_all();
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

    for (uint8_t i = 0; i < lineSensors.getCount(); ++i)
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
    const int16_t center =
        static_cast<int16_t>(lineSensors.getCount() - 1) / 2;

    for (uint8_t i = 0; i < lineSensors.getCount(); ++i)
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
    const uint32_t now = millis();

    if (now - lastControlMs < CONTROL_INTERVAL_MS)
    {
        return;
    }

    lastControlMs = now;

    if (!motorReady)
    {
        stopRobot();
        return;
    }

    int16_t error = 0;
    if (readLineError(error))
    {
        float dt = static_cast<float>(now - lastLineMs) / 1000.0f;

        if (!haveLastLineError || lastLineMs == 0 || dt <= 0.0f)
        {
            dt = 0.0f;
        }

        const float derivative = dt > 0.0f
            ? (static_cast<float>(error) - lastLineError) / dt
            : 0.0f;

        lastTurn = clampTurn(
            LINE_TURN_DIRECTION
            * (error * LINE_TURN_KP + derivative * LINE_TURN_KD)
        );

        robot.drive(LINE_SPEED, 0, lastTurn);
        lastLineError = error;
        lastLineMs = now;
        haveLastLineError = true;
        return;
    }

    haveLastLineError = false;
    lastTurn = lastTurn < 0 ? -SEARCH_TURN_SPEED : SEARCH_TURN_SPEED;
    robot.drive(0, 0, lastTurn);
}

void printStatus()
{
    const uint32_t now = millis();

    if (now - lastStatusMs < STATUS_INTERVAL_MS)
    {
        return;
    }

    Serial.print("[line] motor=");
    Serial.print(motorReady ? "ok" : "fail");
    Serial.print(" last_error=");
    Serial.print(lastLineError);
    Serial.print(" turn=");
    Serial.println(lastTurn);

    lastStatusMs = now;
}
} // namespace

void systemSetup()
{
    Serial.begin(SERIAL_BAUD);

    lineSensors.begin();
    motorReady = robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    stopRobot();

    Serial.println("Line follow only.");
    Serial.println(motorReady ? "Motoron init OK." : "Motoron init FAILED.");
}

void systemLoop()
{
    followLine();
    printStatus();
}

#else

void systemSetup()
{
    Serial.begin(115200);
}

void systemLoop()
{
    delay(10);
}

#endif
