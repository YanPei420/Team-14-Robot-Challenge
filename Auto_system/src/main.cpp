#include <Arduino.h>
#include <Wire.h>

#include "MotorConfig.h"
#include "MotorEncoder.h"
#include "MotoronDrive.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr int16_t TEST_SPEED = 250;
constexpr uint32_t TEST_DURATION_MS = 3000;
constexpr uint32_t COMMAND_REFRESH_MS = 100;
constexpr uint32_t ENCODER_PRINT_MS = 250;

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
MotoronDriveEncoders encoders;

uint8_t activeTest = 0;
const char* activeLabel = "idle";
uint32_t testStartedAt = 0;
uint32_t lastCommandAt = 0;
uint32_t lastEncoderPrintAt = 0;

void printAddress(uint8_t address)
{
    Serial.print("0x");

    if (address < 0x10)
    {
        Serial.print('0');
    }

    Serial.print(address, HEX);
}

void scanWire1()
{
    uint8_t found = 0;

    Serial.println();
    Serial.println("Wire1 I2C scan");

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire1.beginTransmission(address);

        if (Wire1.endTransmission() == 0)
        {
            Serial.print("  found ");
            printAddress(address);
            Serial.println();
            found++;
        }
    }

    Serial.print("Devices found: ");
    Serial.println(found);
}

void printHelp()
{
    Serial.println();
    Serial.println("Chassis motor encoder test");
    Serial.println("i: scan Motoron I2C bus on Wire1");
    Serial.println("p: print Motoron status");
    Serial.println("c: print encoder counts/revolutions/RPM");
    Serial.println("z: reset encoder counts");
    Serial.println("1: front-left wheel");
    Serial.println("2: front-right wheel");
    Serial.println("3: rear-left wheel");
    Serial.println("4: rear-right wheel");
    Serial.println("f: chassis forward");
    Serial.println("x: chassis backward");
    Serial.println("l: chassis left strafe");
    Serial.println("r: chassis right strafe");
    Serial.println("u: chassis rotate left");
    Serial.println("o: chassis rotate right");
    Serial.println("s: stop");
    Serial.println("h or ?: help");
}

void printCachedSpeeds()
{
    int16_t frontLeft = 0;
    int16_t frontRight = 0;
    int16_t rearLeft = 0;
    int16_t rearRight = 0;

    robot.get_wheel_speeds(frontLeft, frontRight, rearLeft, rearRight);

    Serial.print("Command FL/FR/RL/RR: ");
    Serial.print(frontLeft);
    Serial.print(", ");
    Serial.print(frontRight);
    Serial.print(", ");
    Serial.print(rearLeft);
    Serial.print(", ");
    Serial.println(rearRight);
}

void printEncoderState()
{
    int32_t countFrontLeft = 0;
    int32_t countFrontRight = 0;
    int32_t countRearLeft = 0;
    int32_t countRearRight = 0;
    float revFrontLeft = 0.0f;
    float revFrontRight = 0.0f;
    float revRearLeft = 0.0f;
    float revRearRight = 0.0f;
    float rpmFrontLeft = 0.0f;
    float rpmFrontRight = 0.0f;
    float rpmRearLeft = 0.0f;
    float rpmRearRight = 0.0f;

    encoders.get_counts(
        countFrontLeft,
        countFrontRight,
        countRearLeft,
        countRearRight
    );
    encoders.get_revolutions(
        revFrontLeft,
        revFrontRight,
        revRearLeft,
        revRearRight
    );
    encoders.sample_rpm(
        rpmFrontLeft,
        rpmFrontRight,
        rpmRearLeft,
        rpmRearRight
    );

    Serial.print("Encoder count FL/FR/RL/RR: ");
    Serial.print(countFrontLeft);
    Serial.print(", ");
    Serial.print(countFrontRight);
    Serial.print(", ");
    Serial.print(countRearLeft);
    Serial.print(", ");
    Serial.println(countRearRight);

    Serial.print("Encoder rev FL/FR/RL/RR: ");
    Serial.print(revFrontLeft, 3);
    Serial.print(", ");
    Serial.print(revFrontRight, 3);
    Serial.print(", ");
    Serial.print(revRearLeft, 3);
    Serial.print(", ");
    Serial.println(revRearRight, 3);

    Serial.print("Encoder RPM FL/FR/RL/RR: ");
    Serial.print(rpmFrontLeft, 2);
    Serial.print(", ");
    Serial.print(rpmFrontRight, 2);
    Serial.print(", ");
    Serial.print(rpmRearLeft, 2);
    Serial.print(", ");
    Serial.println(rpmRearRight, 2);
}

void applyActiveTest()
{
    switch (activeTest)
    {
        case 1:
            robot.set_front_left(TEST_SPEED);
            break;
        case 2:
            robot.set_front_right(TEST_SPEED);
            break;
        case 3:
            robot.set_rear_left(TEST_SPEED);
            break;
        case 4:
            robot.set_rear_right(TEST_SPEED);
            break;
        case 5:
            robot.forward(TEST_SPEED);
            break;
        case 6:
            robot.backward(TEST_SPEED);
            break;
        case 7:
            robot.left(TEST_SPEED);
            break;
        case 8:
            robot.right(TEST_SPEED);
            break;
        case 9:
            robot.rotate_left(TEST_SPEED);
            break;
        case 10:
            robot.rotate_right(TEST_SPEED);
            break;
        default:
            break;
    }
}

void startTest(uint8_t testNumber, const char* label)
{
    robot.stop();
    robot.clear_status_flags();
    encoders.reset_counts();

    activeTest = testNumber;
    activeLabel = label;
    testStartedAt = millis();
    lastCommandAt = 0;
    lastEncoderPrintAt = 0;

    Serial.println();
    Serial.print("Start ");
    Serial.println(activeLabel);

    applyActiveTest();
    printCachedSpeeds();
    printEncoderState();
}

void stopTest()
{
    activeTest = 0;
    activeLabel = "idle";
    robot.stop();

    Serial.println();
    Serial.println("Stop");
    printCachedSpeeds();
    printEncoderState();
    robot.print_status(Serial);
}

void handleCommand(char command)
{
    switch (command)
    {
        case 'i':
            scanWire1();
            break;
        case 'p':
            robot.clear_status_flags();
            robot.print_status(Serial);
            break;
        case 'c':
            printEncoderState();
            break;
        case 'z':
            encoders.reset_counts();
            Serial.println("Encoder counts reset");
            printEncoderState();
            break;
        case '1':
            startTest(1, "front-left wheel");
            break;
        case '2':
            startTest(2, "front-right wheel");
            break;
        case '3':
            startTest(3, "rear-left wheel");
            break;
        case '4':
            startTest(4, "rear-right wheel");
            break;
        case 'f':
            startTest(5, "chassis forward");
            break;
        case 'x':
            startTest(6, "chassis backward");
            break;
        case 'l':
            startTest(7, "chassis left strafe");
            break;
        case 'r':
            startTest(8, "chassis right strafe");
            break;
        case 'u':
            startTest(9, "chassis rotate left");
            break;
        case 'o':
            startTest(10, "chassis rotate right");
            break;
        case 's':
            stopTest();
            break;
        case 'h':
        case '?':
            printHelp();
            break;
        case '\r':
        case '\n':
            break;
        default:
            Serial.print("Unknown command: ");
            Serial.println(command);
            printHelp();
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
    const bool encoderReady = encoders.begin();

    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.stop();

    Serial.println();
    Serial.println("Chassis motor encoder test ready");
    Serial.print("Motoron init: ");
    Serial.println(motoronReady ? "OK" : "check status/errors");
    Serial.print("Encoder init: ");
    Serial.println(encoderReady ? "OK" : "check interrupt pins");
    Serial.print("Counts per output revolution: ");
    Serial.println(MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV);

    robot.print_status(Serial);
    printHelp();
}

void loop()
{
    const uint32_t now = millis();

    if (activeTest != 0)
    {
        if (now - testStartedAt >= TEST_DURATION_MS)
        {
            stopTest();
        }
        else
        {
            if (now - lastCommandAt >= COMMAND_REFRESH_MS)
            {
                applyActiveTest();
                lastCommandAt = now;
            }

            if (now - lastEncoderPrintAt >= ENCODER_PRINT_MS)
            {
                printEncoderState();
                lastEncoderPrintAt = now;
            }
        }
    }

    while (Serial.available())
    {
        handleCommand(static_cast<char>(Serial.read()));
    }
}
