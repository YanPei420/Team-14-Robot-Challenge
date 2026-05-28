#include <Arduino.h>
#include <Wire.h>

#include "MotoronDrive.h"

MotoronDrive Robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

constexpr uint32_t SERIAL_WAIT_MS = 3000;
constexpr uint32_t COMMAND_PERIOD_MS = 100;
constexpr uint32_t STATUS_PERIOD_MS = 1000;
constexpr int16_t TEST_SPEED = 300;

bool motoronReady = false;
int16_t activeRotation = 0;
uint32_t lastCommandMs = 0;
uint32_t lastStatusMs = 0;

void waitForSerial(uint32_t timeoutMs)
{
    const uint32_t startMs = millis();

    while (!Serial && millis() - startMs < timeoutMs)
    {
        delay(10);
    }
}

bool i2cAddressPresent(uint8_t address)
{
    Wire1.beginTransmission(address);
    return Wire1.endTransmission() == 0;
}

void printWire1Scan()
{
    Serial.println("Scanning Wire1 I2C bus...");

    uint8_t found = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire1.beginTransmission(address);
        const uint8_t error = Wire1.endTransmission();

        if (error == 0)
        {
            Serial.print("  Found 0x");
            if (address < 16)
            {
                Serial.print('0');
            }
            Serial.println(address, HEX);
            found++;
        }
    }

    if (found == 0)
    {
        Serial.println("  No Wire1 I2C devices found.");
    }
}

void printHelp()
{
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  l: rotate left");
    Serial.println("  r: rotate right");
    Serial.println("  s: stop");
    Serial.println("  ?: help");
    Serial.println();
}

void handleSerialCommand()
{
    if (!Serial.available())
    {
        return;
    }

    const char command = Serial.read();

    switch (command)
    {
    case 'l':
    case 'L':
        activeRotation = -TEST_SPEED;
        Serial.println("Rotate left.");
        break;

    case 'r':
    case 'R':
        activeRotation = TEST_SPEED;
        Serial.println("Rotate right.");
        break;

    case 's':
    case 'S':
        activeRotation = 0;
        Robot.stop_all();
        Serial.println("Stopped.");
        break;

    case '?':
    case 'h':
    case 'H':
        printHelp();
        break;

    default:
        break;
    }
}

void refreshMotorCommand()
{
    const uint32_t now = millis();

    if (now - lastCommandMs < COMMAND_PERIOD_MS)
    {
        return;
    }

    lastCommandMs = now;

    if (activeRotation < 0)
    {
        Robot.rotate_left(-activeRotation);
    }
    else if (activeRotation > 0)
    {
        Robot.rotate_right(activeRotation);
    }
    else
    {
        Robot.stop_all();
    }
}

void printStatusPeriodically()
{
    const uint32_t now = millis();

    if (now - lastStatusMs < STATUS_PERIOD_MS)
    {
        return;
    }

    lastStatusMs = now;
    Robot.print_status(Serial);
}

void setup()
{
    Serial.begin(115200);
    waitForSerial(SERIAL_WAIT_MS);

    Serial.println();
    Serial.println("GIGA Motoron diagnostic boot");

    Wire1.begin();
    printWire1Scan();

    const bool frontSeen = i2cAddressPresent(MOTORON_ADDR_FRONT);
    const bool rearSeen = i2cAddressPresent(MOTORON_ADDR_REAR);

    Serial.print("Front Motoron 0x");
    Serial.print(MOTORON_ADDR_FRONT, HEX);
    Serial.println(frontSeen ? " detected" : " NOT detected");

    Serial.print("Rear Motoron 0x");
    Serial.print(MOTORON_ADDR_REAR, HEX);
    Serial.println(rearSeen ? " detected" : " NOT detected");

    if (!frontSeen || !rearSeen)
    {
        Serial.println("Motoron missing on Wire1. Motors will stay stopped.");
        return;
    }

    motoronReady = Robot.begin();
    Serial.print("Motoron init: ");
    Serial.println(motoronReady ? "OK" : "FAILED");
    Robot.print_status(Serial);

    if (!motoronReady)
    {
        Robot.stop_all();
        Serial.println("Motoron init failed. Motors will stay stopped.");
        return;
    }

    Robot.clear_status_flags();
    Robot.stop_all();
    printHelp();
}

void loop()
{
    if (!motoronReady)
    {
        delay(100);
        return;
    }

    handleSerialCommand();
    refreshMotorCommand();
    printStatusPeriodically();
}
