#include <Arduino.h>
#include <Wire.h>

#include "MotorConfig.h"
#include "MotoronDrive.h"

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr int16_t TEST_SPEED = 250;
constexpr uint32_t TEST_DURATION_MS = 2000;
constexpr uint32_t COMMAND_REFRESH_MS = 100;

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

int activeTest = 0;
uint32_t testStartedAt = 0;
uint32_t lastCommandAt = 0;

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
    Serial.println("Layer 1: scan Wire1");

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
    Serial.println("Motor layer test");
    Serial.println("i: layer 1 scan Wire1");
    Serial.println("p: layer 2 print Motoron status");
    Serial.println("f: layer 3 raw front board both channels");
    Serial.println("b: layer 3 raw rear board both channels");
    Serial.println("1: layer 4 front-left wheel");
    Serial.println("2: layer 4 front-right wheel");
    Serial.println("3: layer 4 rear-left wheel");
    Serial.println("4: layer 4 rear-right wheel");
    Serial.println("q: front raw M1 setSpeedNow");
    Serial.println("w: front raw M2 setSpeedNow");
    Serial.println("r: layer 5 chassis right strafe");
    Serial.println("s: stop");
}

void applyActiveTest()
{
    switch (activeTest)
    {
        case 1:
            robot.raw_front(TEST_SPEED, TEST_SPEED);
            break;
        case 2:
            robot.raw_rear(TEST_SPEED, TEST_SPEED);
            break;
        case 3:
            robot.set_front_left(TEST_SPEED);
            break;
        case 4:
            robot.set_front_right(TEST_SPEED);
            break;
        case 5:
            robot.set_rear_left(TEST_SPEED);
            break;
        case 6:
            robot.set_rear_right(TEST_SPEED);
            break;
        case 7:
            robot.right(TEST_SPEED);
            break;
        case 8:
            robot.raw_front_motor(1, TEST_SPEED, true);
            break;
        case 9:
            robot.raw_front_motor(2, TEST_SPEED, true);
            break;
        case 10:
            break;
        default:
            break;
    }
}

void startTest(int testNumber, const char* label)
{
    robot.stop();
    robot.clear_status_flags();
    activeTest = testNumber;
    testStartedAt = millis();
    lastCommandAt = 0;

    Serial.print("Start ");
    Serial.println(label);

    applyActiveTest();
    robot.print_status(Serial);
}

void stopTest()
{
    activeTest = 0;
    robot.stop();
    Serial.println("Stop");
    robot.print_status(Serial);
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

    printHelp();
}

void loop()
{
    if (activeTest != 0)
    {
        if (millis() - testStartedAt >= TEST_DURATION_MS)
        {
            stopTest();
        }
        else if (millis() - lastCommandAt >= COMMAND_REFRESH_MS)
        {
            applyActiveTest();
            lastCommandAt = millis();
        }
    }

    if (!Serial.available())
    {
        return;
    }

    const char command = Serial.read();

    switch (command)
    {
        case 'i':
            scanWire1();
            break;
        case 'p':
            robot.clear_status_flags();
            robot.print_status(Serial);
            break;
        case 'f':
            startTest(1, "raw front board");
            break;
        case 'b':
            startTest(2, "raw rear board");
            break;
        case '1':
            startTest(3, "front-left wheel");
            break;
        case '2':
            startTest(4, "front-right wheel");
            break;
        case '3':
            startTest(5, "rear-left wheel");
            break;
        case '4':
            startTest(6, "rear-right wheel");
            break;
        case 'q':
            startTest(8, "front raw M1 setSpeedNow");
            break;
        case 'w':
            startTest(9, "front raw M2 setSpeedNow");
            break;
        case 'r':
            startTest(7, "right strafe");
            break;
        case 's':
            stopTest();
            break;
        case 'h':
        case '?':
            printHelp();
            break;
        default:
            break;
    }
}
