#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(
    TwoWire& wireBus,
    uint8_t sensorAddress
)
{
    wire = &wireBus;

    address = sensorAddress;
}

void DistanceSensor::begin()
{
    wire->begin();

    delay(500);
}

float DistanceSensor::readDistanceCM()
{
    // ============================================
    // READ DISTANCE REGISTER
    // 0x5E = Distance[11:4]
    // 0x5F = Distance[3:0]
    // ============================================

    wire->beginTransmission(address);

    wire->write(0x5E);

    wire->endTransmission(false);

    wire->requestFrom(address, (uint8_t)2);

    if (wire->available() < 2)
    {
        return -1.0f;
    }

    uint8_t highByte = wire->read();
    uint8_t lowByte  = wire->read();

    // ============================================
    // DISTANCE CALCULATION
    // Datasheet:
    // Distance = (high*16 + low)/16/2^n
    //
    // Default Shift Bit:
    // n = 2
    // ============================================

    uint16_t raw =
        ((uint16_t)highByte << 4)
        |
        (lowByte & 0x0F);

    float distance =
        raw
        /
        16.0f
        /
        4.0f;

    return distance;
}