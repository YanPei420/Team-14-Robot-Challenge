#include <Arduino.h>

#include "IRConfig.h"
#include "IRSensor.h"
#include "MotorConfig.h"
#include "MotorEncoder.h"
#include "MotoronDrive.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t DEBUG_INTERVAL_MS = 250;
constexpr uint32_t CONTROL_INTERVAL_MS = 50;

constexpr int16_t LINE_SPEED = 200;
constexpr int16_t SEARCH_TURN_SPEED = 180;
constexpr int16_t MAX_TURN_SPEED = 800;

constexpr int16_t SENSOR_STEP = 1000;
constexpr uint16_t LINE_MIN_CONTRAST = 50;
constexpr float LINE_TURN_KP = 0.08f;
constexpr float LINE_TURN_KD = 0.003f;

constexpr float WHEEL_TARGET_RPM_PER_COMMAND = 0.25f;
constexpr float WHEEL_PD_KP = 1.2f;
constexpr float WHEEL_PD_KD = 0.02f;
constexpr int16_t MAX_WHEEL_CORRECTION = 250;

struct WheelCommands
{
    int16_t frontLeft;
    int16_t frontRight;
    int16_t rearLeft;
    int16_t rearRight;
};

struct WheelFeedback
{
    float frontLeftRpm;
    float frontRightRpm;
    float rearLeftRpm;
    float rearRightRpm;
    int16_t frontLeftCorrection;
    int16_t frontRightCorrection;
    int16_t rearLeftCorrection;
    int16_t rearRightCorrection;
};

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
MotoronDriveEncoders encoders;
IRSensor lineSensors(IR_PINS, IR_SENSOR_COUNT);

bool encodersReady = false;
bool haveLastLineError = false;
int16_t lastTurn = 0;
int16_t lastLineError = 0;
WheelCommands lastTarget = {0, 0, 0, 0};
WheelFeedback lastFeedback = {0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0};
float previousWheelErrors[4] = {0.0f, 0.0f, 0.0f, 0.0f};
uint32_t lastControlAt = 0;
uint32_t lastLineSampleAt = 0;
uint32_t lastEncoderSampleAt = 0;
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

int16_t clampWheelCommand(int32_t command)
{
    if (command > MOTOR_MAX_SPEED)
    {
        return MOTOR_MAX_SPEED;
    }

    if (command < -MOTOR_MAX_SPEED)
    {
        return -MOTOR_MAX_SPEED;
    }

    return static_cast<int16_t>(command);
}

int16_t clampCorrection(float correction)
{
    if (correction > MAX_WHEEL_CORRECTION)
    {
        return MAX_WHEEL_CORRECTION;
    }

    if (correction < -MAX_WHEEL_CORRECTION)
    {
        return -MAX_WHEEL_CORRECTION;
    }

    return static_cast<int16_t>(correction);
}

int32_t absInt32(int32_t value)
{
    return value < 0 ? -value : value;
}

WheelCommands mixDrive(int16_t vx, int16_t vy, int16_t w)
{
    int32_t frontLeft = static_cast<int32_t>(vx) - vy - w;
    int32_t frontRight = static_cast<int32_t>(vx) + vy + w;
    int32_t rearLeft = static_cast<int32_t>(vx) + vy - w;
    int32_t rearRight = static_cast<int32_t>(vx) - vy + w;

    int32_t largest = absInt32(frontLeft);

    if (absInt32(frontRight) > largest)
    {
        largest = absInt32(frontRight);
    }

    if (absInt32(rearLeft) > largest)
    {
        largest = absInt32(rearLeft);
    }

    if (absInt32(rearRight) > largest)
    {
        largest = absInt32(rearRight);
    }

    if (largest > MOTOR_MAX_SPEED && largest != 0)
    {
        frontLeft = frontLeft * MOTOR_MAX_SPEED / largest;
        frontRight = frontRight * MOTOR_MAX_SPEED / largest;
        rearLeft = rearLeft * MOTOR_MAX_SPEED / largest;
        rearRight = rearRight * MOTOR_MAX_SPEED / largest;
    }

    return {
        static_cast<int16_t>(frontLeft),
        static_cast<int16_t>(frontRight),
        static_cast<int16_t>(rearLeft),
        static_cast<int16_t>(rearRight)
    };
}

int16_t calculateWheelCommand(
    uint8_t wheelIndex,
    int16_t targetCommand,
    float measuredRpm,
    float dtSeconds,
    int16_t& correction
)
{
    if (!encodersReady || targetCommand == 0 || dtSeconds <= 0.0f)
    {
        previousWheelErrors[wheelIndex] = 0.0f;
        correction = 0;
        return targetCommand;
    }

    const float targetRpm =
        static_cast<float>(targetCommand) * WHEEL_TARGET_RPM_PER_COMMAND;
    const float error = targetRpm - measuredRpm;
    const float derivative =
        (error - previousWheelErrors[wheelIndex]) / dtSeconds;

    previousWheelErrors[wheelIndex] = error;
    correction = clampCorrection(
        WHEEL_PD_KP * error + WHEEL_PD_KD * derivative
    );

    return clampWheelCommand(static_cast<int32_t>(targetCommand) + correction);
}

void driveWithEncoderPd(const WheelCommands& target)
{
    lastTarget = target;

    const uint32_t now = millis();
    float dtSeconds = static_cast<float>(now - lastEncoderSampleAt) / 1000.0f;
    if (lastEncoderSampleAt == 0)
    {
        dtSeconds = 0.0f;
    }
    lastEncoderSampleAt = now;

    encoders.sample_rpm(
        lastFeedback.frontLeftRpm,
        lastFeedback.frontRightRpm,
        lastFeedback.rearLeftRpm,
        lastFeedback.rearRightRpm
    );

    const int16_t frontLeft = calculateWheelCommand(
        0,
        target.frontLeft,
        lastFeedback.frontLeftRpm,
        dtSeconds,
        lastFeedback.frontLeftCorrection
    );
    const int16_t frontRight = calculateWheelCommand(
        1,
        target.frontRight,
        lastFeedback.frontRightRpm,
        dtSeconds,
        lastFeedback.frontRightCorrection
    );
    const int16_t rearLeft = calculateWheelCommand(
        2,
        target.rearLeft,
        lastFeedback.rearLeftRpm,
        dtSeconds,
        lastFeedback.rearLeftCorrection
    );
    const int16_t rearRight = calculateWheelCommand(
        3,
        target.rearRight,
        lastFeedback.rearRightRpm,
        dtSeconds,
        lastFeedback.rearRightCorrection
    );

    robot.set_all(frontLeft, frontRight, rearLeft, rearRight);
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
    const uint32_t now = millis();
    if (now - lastControlAt < CONTROL_INTERVAL_MS)
    {
        return;
    }
    lastControlAt = now;

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
            error * LINE_TURN_KP + derivative * LINE_TURN_KD
        );

        driveWithEncoderPd(mixDrive(LINE_SPEED, 0, turn));
        lastTurn = turn;
        lastLineError = error;
        lastLineSampleAt = now;
        haveLastLineError = true;
        return;
    }

    haveLastLineError = false;

    if (lastTurn < 0)
    {
        lastTurn = -SEARCH_TURN_SPEED;
        driveWithEncoderPd(mixDrive(0, 0, lastTurn));
    }
    else
    {
        lastTurn = SEARCH_TURN_SPEED;
        driveWithEncoderPd(mixDrive(0, 0, lastTurn));
    }
}

void printDebug()
{
    int32_t frontLeftCount = 0;
    int32_t frontRightCount = 0;
    int32_t rearLeftCount = 0;
    int32_t rearRightCount = 0;

    encoders.get_counts(
        frontLeftCount,
        frontRightCount,
        rearLeftCount,
        rearRightCount
    );

    Serial.print("IR:");

    for (uint8_t i = 0; i < lineSensors.getCount(); i++)
    {
        Serial.print(' ');
        Serial.print(lineSensors.getValue(i));
    }

    Serial.print(" | err=");
    Serial.print(lastLineError);
    Serial.print(" | turn=");
    Serial.print(lastTurn);
    Serial.print(" | target FL/FR/RL/RR=");
    Serial.print(lastTarget.frontLeft);
    Serial.print('/');
    Serial.print(lastTarget.frontRight);
    Serial.print('/');
    Serial.print(lastTarget.rearLeft);
    Serial.print('/');
    Serial.print(lastTarget.rearRight);
    Serial.print(" | rpm FL/FR/RL/RR=");
    Serial.print(lastFeedback.frontLeftRpm, 1);
    Serial.print('/');
    Serial.print(lastFeedback.frontRightRpm, 1);
    Serial.print('/');
    Serial.print(lastFeedback.rearLeftRpm, 1);
    Serial.print('/');
    Serial.print(lastFeedback.rearRightRpm, 1);
    Serial.print(" | pd FL/FR/RL/RR=");
    Serial.print(lastFeedback.frontLeftCorrection);
    Serial.print('/');
    Serial.print(lastFeedback.frontRightCorrection);
    Serial.print('/');
    Serial.print(lastFeedback.rearLeftCorrection);
    Serial.print('/');
    Serial.print(lastFeedback.rearRightCorrection);
    Serial.print(" | enc FL/FR/RL/RR=");
    Serial.print(frontLeftCount);
    Serial.print('/');
    Serial.print(frontRightCount);
    Serial.print('/');
    Serial.print(rearLeftCount);
    Serial.print('/');
    Serial.println(rearRightCount);
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

    encodersReady = encoders.begin();
    encoders.reset_counts();
    lastEncoderSampleAt = millis();

    Serial.println("Line following started.");
    Serial.print("Motoron init: ");
    Serial.println(motoronReady ? "OK" : "check wiring or I2C address");
    Serial.print("Encoder init: ");
    Serial.println(encodersReady ? "OK" : "check interrupt-capable pins");
    Serial.print("Counts per output revolution: ");
    Serial.println(MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV);
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
