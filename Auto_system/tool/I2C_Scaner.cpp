#include <Arduino.h>
#include <Wire.h>

#include "MotorConfig.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;

void printAddress(uint8_t address)
{
    Serial.print("0x");
    if (address < 0x10)
    {
        Serial.print('0');
    }
    Serial.print(address, HEX);
    Serial.print(" (");
    Serial.print(address);
    Serial.print(')');
}

uint8_t probeAddress(TwoWire& bus, uint8_t address)
{
    bus.beginTransmission(address);
    return bus.endTransmission();
}

void scanI2C(TwoWire& bus, const char* busName)
{
    uint8_t found = 0;

    Serial.print("\nScanning ");
    Serial.print(busName);
    Serial.println("...");

    for (uint8_t address = 1; address < 127; address++)
    {
        const uint8_t error = probeAddress(bus, address);

        if (error == 0)
        {
            Serial.print("  Found device at ");
            printAddress(address);
            Serial.println();
            found++;
        }
        else if (error == 4)
        {
            Serial.print("  Unknown error at ");
            printAddress(address);
            Serial.println();
        }
    }

    Serial.print("Scan complete. Devices found: ");
    Serial.println(found);
}

void checkAddress(TwoWire& bus, const char* busName, uint8_t address)
{
    Serial.print(busName);
    Serial.print(" ");
    printAddress(address);
    Serial.print(": ");

    const uint8_t error = probeAddress(bus, address);
    switch (error)
    {
        case 0:
            Serial.println("ACK");
            break;
        case 2:
            Serial.println("NACK on address");
            break;
        case 3:
            Serial.println("NACK on data");
            break;
        case 4:
            Serial.println("other error");
            break;
        default:
            Serial.print("error ");
            Serial.println(error);
            break;
    }
}

void checkMotoronAddresses()
{
    Serial.println("\nChecking expected Motoron addresses on Wire1...");
    checkAddress(Wire1, "Wire1", MOTORON_ADDR_FRONT);
    checkAddress(Wire1, "Wire1", MOTORON_ADDR_REAR);
}

void printHelp()
{
    Serial.println("\nI2C Debug Tool");
    Serial.println("Commands:");
    Serial.println("  1 - scan Wire1");
    Serial.println("  0 - scan Wire");
    Serial.println("  a - scan both buses");
    Serial.println("  m - check Motoron addresses 0x10 and 0x11 on Wire1");
    Serial.println("  h - show this menu");
}
} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);

    while (!Serial)
    {
        ;
    }

    Wire.begin();
    Wire1.begin();

    printHelp();
}

void loop()
{
    if (!Serial.available())
    {
        return;
    }

    const char command = Serial.read();

    switch (command)
    {
        case '1':
        case 's':
            scanI2C(Wire1, "Wire1");
            break;
        case '0':
            scanI2C(Wire, "Wire");
            break;
        case 'a':
            scanI2C(Wire, "Wire");
            scanI2C(Wire1, "Wire1");
            break;
        case 'm':
            checkMotoronAddresses();
            break;
        case 'h':
        case '?':
            printHelp();
            break;
        default:
            break;
    }
}
