#include <Arduino.h>
#include <Motoron.h>
#include <Wire.h>

#include "MotorConfig.h"
#include "MotorEncoder.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t PRINT_INTERVAL_MS = 500;
constexpr uint32_t COMMAND_REFRESH_MS = 100;

constexpr uint8_t MOTORON_ADDR = MOTORON_ADDR_FRONT;       // 0x10 / 16
constexpr uint8_t MOTORON_M1_CHANNEL = 1;
constexpr int16_t TEST_SPEED = 250;

MotoronI2C motoron(MOTORON_ADDR);
MotorEncoder encoder(
    MOTOR_ENCODER_FRONT_LEFT_A_PIN,
    MOTOR_ENCODER_FRONT_LEFT_B_PIN,
    MOTOR_ENCODER_FRONT_LEFT_DIRECTION,
    MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV
);

int16_t commandedSpeed = 0;
uint32_t lastPrintAt = 0;
uint32_t lastCommandAt = 0;

void printHelp()
{
    Serial.println();
    Serial.println("Single Motoron M1 encoder test");
    Serial.println("Target: Motoron address 0x10 / 16, channel M1");
    Serial.println("Encoder pins: A=D22, B=D23");
    Serial.println();
    Serial.println("f: run M1 forward");
    Serial.println("b: run M1 backward");
    Serial.println("s: stop M1");
    Serial.println("r: reset encoder count");
    Serial.println("p: print one sample now");
    Serial.println("h or ?: print help");
    Serial.println();
}

void setupMotoron()
{
    Wire1.begin();

    motoron.setBus(&Wire1);
    motoron.reinitialize();
    motoron.disableCrc();
    motoron.clearResetFlag();
    motoron.clearLatchedStatusFlags(0xFFFF);
    motoron.clearMotorFaultUnconditional();
    motoron.setCommandTimeoutMilliseconds(MOTOR_COMMAND_TIMEOUT_MS);
    motoron.setMaxAcceleration(MOTORON_M1_CHANNEL, MOTOR_MAX_ACCELERATION);
    motoron.setMaxDeceleration(MOTORON_M1_CHANNEL, MOTOR_MAX_DECELERATION);
    motoron.setSpeedNow(MOTORON_M1_CHANNEL, 0);
}

void setMotorSpeed(int16_t speed)
{
    if (speed > MOTOR_MAX_SPEED)
    {
        speed = MOTOR_MAX_SPEED;
    }
    else if (speed < -MOTOR_MAX_SPEED)
    {
        speed = -MOTOR_MAX_SPEED;
    }

    commandedSpeed = speed;
    motoron.setSpeed(MOTORON_M1_CHANNEL, commandedSpeed);
    lastCommandAt = millis();
}

void stopMotor()
{
    commandedSpeed = 0;
    motoron.setSpeedNow(MOTORON_M1_CHANNEL, 0);
    lastCommandAt = millis();
}

void printSample()
{
    const int32_t count = encoder.read_count();
    const float revolutions = encoder.read_revolutions();
    const float rpm = encoder.sample_rpm();

    Serial.print("ms=");
    Serial.print(millis());
    Serial.print(" | cmd=");
    Serial.print(commandedSpeed);
    Serial.print(" | count=");
    Serial.print(count);
    Serial.print(" | rev=");
    Serial.print(revolutions, 3);
    Serial.print(" | rpm=");
    Serial.print(rpm, 1);
    Serial.print(" | motoronLastError=");
    Serial.println(motoron.getLastError());
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
        case 'f':
            encoder.reset_count();
            setMotorSpeed(TEST_SPEED);
            Serial.println("M1 forward.");
            break;
        case 'b':
            encoder.reset_count();
            setMotorSpeed(-TEST_SPEED);
            Serial.println("M1 backward.");
            break;
        case 's':
            stopMotor();
            Serial.println("M1 stopped.");
            printSample();
            break;
        case 'r':
            encoder.reset_count();
            Serial.println("Encoder count reset.");
            printSample();
            break;
        case 'p':
            printSample();
            break;
        case 'h':
        case '?':
            printHelp();
            break;
        default:
            break;
    }
}

void refreshMotorCommand()
{
    if (commandedSpeed == 0)
    {
        return;
    }

    if (millis() - lastCommandAt >= COMMAND_REFRESH_MS)
    {
        motoron.setSpeed(MOTORON_M1_CHANNEL, commandedSpeed);
        lastCommandAt = millis();
    }
}
}

void setup()
{
    Serial.begin(SERIAL_BAUD);

    const uint32_t serialStartMs = millis();
    while (!Serial && millis() - serialStartMs < 3000)
    {
        ;
    }

    setupMotoron();

    const bool encoderReady = encoder.begin(0);
    encoder.reset_count();

    Serial.println("Single encoder test started.");
    Serial.print("Motoron address: 0x");
    Serial.print(MOTORON_ADDR, HEX);
    Serial.println(" / 16");
    Serial.print("Motoron channel: M1 / ");
    Serial.println(MOTORON_M1_CHANNEL);
    Serial.print("Encoder init: ");
    Serial.println(encoderReady ? "OK" : "check interrupt-capable pins D22/D23");
    Serial.print("Counts per output revolution: ");
    Serial.println(MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV);
    Serial.print("Motoron lastError: ");
    Serial.println(motoron.getLastError());

    printHelp();
}

void loop()
{
    handleSerial();
    refreshMotorCommand();

    if (millis() - lastPrintAt >= PRINT_INTERVAL_MS)
    {
        printSample();
        lastPrintAt = millis();
    }
}
