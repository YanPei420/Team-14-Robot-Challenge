#include <Arduino.h>
#include <Wire.h>

#include "MotorConfig.h"
#include "MotorEncoder.h"
#include "MotoronDrive.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t PRINT_INTERVAL_MS = 500;
constexpr uint32_t MOTOR_COMMAND_REFRESH_MS = 100;
constexpr uint32_t MOTOR_TEST_DURATION_MS = 2000;
constexpr int16_t MOTOR_TEST_SPEED = 250;

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
MotoronDriveEncoders encoders;

uint8_t activeMotorTest = 0;
uint32_t motorTestStartedAt = 0;
uint32_t lastMotorCommandAt = 0;
uint32_t lastPrintAt = 0;

void printHelp()
{
    Serial.println();
    Serial.println("Encoder test");
    Serial.println("h or ?: print help");
    Serial.println("p: print one encoder sample now");
    Serial.println("r: reset all encoder counts");
    Serial.println("s: stop motor test");
    Serial.println("1: drive front-left wheel briefly");
    Serial.println("2: drive front-right wheel briefly");
    Serial.println("3: drive rear-left wheel briefly");
    Serial.println("4: drive rear-right wheel briefly");
    Serial.println("f: drive all wheels forward briefly");
    Serial.println("b: drive all wheels backward briefly");
    Serial.println();
    Serial.println("Output columns: ms, FL count/rev/rpm, FR count/rev/rpm, RL count/rev/rpm, RR count/rev/rpm");
}

void printWheel(const char* label, int32_t count, float revolutions, float rpm)
{
    Serial.print(label);
    Serial.print(" count=");
    Serial.print(count);
    Serial.print(" rev=");
    Serial.print(revolutions, 3);
    Serial.print(" rpm=");
    Serial.print(rpm, 1);
}

void printEncoderSample()
{
    int32_t frontLeftCount = 0;
    int32_t frontRightCount = 0;
    int32_t rearLeftCount = 0;
    int32_t rearRightCount = 0;

    float frontLeftRev = 0.0f;
    float frontRightRev = 0.0f;
    float rearLeftRev = 0.0f;
    float rearRightRev = 0.0f;

    float frontLeftRpm = 0.0f;
    float frontRightRpm = 0.0f;
    float rearLeftRpm = 0.0f;
    float rearRightRpm = 0.0f;

    encoders.get_counts(
        frontLeftCount,
        frontRightCount,
        rearLeftCount,
        rearRightCount
    );

    encoders.get_revolutions(
        frontLeftRev,
        frontRightRev,
        rearLeftRev,
        rearRightRev
    );

    encoders.sample_rpm(
        frontLeftRpm,
        frontRightRpm,
        rearLeftRpm,
        rearRightRpm
    );

    Serial.print("ms=");
    Serial.print(millis());
    Serial.print(" | ");
    printWheel("FL", frontLeftCount, frontLeftRev, frontLeftRpm);
    Serial.print(" | ");
    printWheel("FR", frontRightCount, frontRightRev, frontRightRpm);
    Serial.print(" | ");
    printWheel("RL", rearLeftCount, rearLeftRev, rearLeftRpm);
    Serial.print(" | ");
    printWheel("RR", rearRightCount, rearRightRev, rearRightRpm);
    Serial.println();
}

void applyMotorTest()
{
    switch (activeMotorTest)
    {
        case 1:
            robot.set_front_left(MOTOR_TEST_SPEED);
            break;
        case 2:
            robot.set_front_right(MOTOR_TEST_SPEED);
            break;
        case 3:
            robot.set_rear_left(MOTOR_TEST_SPEED);
            break;
        case 4:
            robot.set_rear_right(MOTOR_TEST_SPEED);
            break;
        case 5:
            robot.forward(MOTOR_TEST_SPEED);
            break;
        case 6:
            robot.backward(MOTOR_TEST_SPEED);
            break;
        default:
            break;
    }
}

void stopMotorTest()
{
    activeMotorTest = 0;
    robot.stop();
    Serial.println("Motor test stopped.");
    printEncoderSample();
}

void startMotorTest(uint8_t testNumber, const char* label)
{
    robot.stop();
    encoders.reset_counts();

    activeMotorTest = testNumber;
    motorTestStartedAt = millis();
    lastMotorCommandAt = 0;

    Serial.print("Start motor test: ");
    Serial.println(label);
    applyMotorTest();
    printEncoderSample();
}

void handleMotorTest()
{
    if (activeMotorTest == 0)
    {
        return;
    }

    if (millis() - motorTestStartedAt >= MOTOR_TEST_DURATION_MS)
    {
        stopMotorTest();
        return;
    }

    if (millis() - lastMotorCommandAt >= MOTOR_COMMAND_REFRESH_MS)
    {
        applyMotorTest();
        lastMotorCommandAt = millis();
    }
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
        case 'h':
        case '?':
            printHelp();
            break;
        case 'p':
            printEncoderSample();
            break;
        case 'r':
            encoders.reset_counts();
            Serial.println("Encoder counts reset.");
            printEncoderSample();
            break;
        case 's':
            stopMotorTest();
            break;
        case '1':
            startMotorTest(1, "front-left");
            break;
        case '2':
            startMotorTest(2, "front-right");
            break;
        case '3':
            startMotorTest(3, "rear-left");
            break;
        case '4':
            startMotorTest(4, "rear-right");
            break;
        case 'f':
            startMotorTest(5, "forward");
            break;
        case 'b':
            startMotorTest(6, "backward");
            break;
        default:
            break;
    }
}
}

void setup()
{
    Serial.begin(SERIAL_BAUD);

    while (!Serial)
    {
        ;
    }

    Wire1.begin();

    const bool motoronReady = robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.stop();

    const bool encodersReady = encoders.begin();
    encoders.reset_counts();

    Serial.println("Encoder test started.");
    Serial.print("Motoron init: ");
    Serial.println(motoronReady ? "OK" : "check wiring or I2C address");
    Serial.print("Encoder init: ");
    Serial.println(encodersReady ? "OK" : "check interrupt-capable pins");
    Serial.print("Counts per output revolution: ");
    Serial.println(MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV);

    printHelp();
}

void loop()
{
    handleMotorTest();
    handleSerial();

    if (millis() - lastPrintAt >= PRINT_INTERVAL_MS)
    {
        printEncoderSample();
        lastPrintAt = millis();
    }
}
