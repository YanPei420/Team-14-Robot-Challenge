#include "LidarSensor.h"

LidarSensor::LidarSensor(HardwareSerial& serialPort)
    : serial(serialPort)
{
    distanceCM         = LIDAR_INVALID_DISTANCE;
    strength           = 0;
    temperature        = 0.0f;
    lastUpdateMs       = 0;
    validFrameCount    = 0;
    checksumErrorCount = 0;
    lowStrengthCount   = 0;
    droppedByteCount   = 0;
}

void LidarSensor::begin()
{
    serial.begin(LIDAR_BAUD_RATE);
}

bool LidarSensor::update()
{
    if (serial.available() < 9) return false;

    // Sync to header byte 0x59 — discard any garbage before it
    while (serial.available() >= 9)
    {
        if (serial.peek() == 0x59) break;
        serial.read();
        ++droppedByteCount;
    }
    if (serial.available() < 9) return false;

    uint8_t raw[9];
    serial.readBytes(raw, 9);

    if (raw[0] != 0x59 || raw[1] != 0x59) return false;

    // Checksum: low byte of sum of bytes 0-7
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) sum += raw[i];
    if ((sum & 0xFF) != raw[8])
    {
        ++checksumErrorCount;
        return false;
    }

    uint16_t amp = raw[4] + raw[5] * 256;
    if (amp < LIDAR_MIN_STRENGTH)
    {
        ++lowStrengthCount;
        return false;
    }

    distanceCM   = raw[2] + raw[3] * 256;
    strength     = amp;
    temperature  = (raw[6] + raw[7] * 256) / 100.0f;
    lastUpdateMs = millis();
    ++validFrameCount;

    return true;
}

int16_t       LidarSensor::getDistanceCM()        { return distanceCM; }
uint16_t      LidarSensor::getStrength()           { return strength; }
float         LidarSensor::getTemperature()        { return temperature; }
unsigned long LidarSensor::getLastUpdateMs()       { return lastUpdateMs; }
uint32_t      LidarSensor::getValidFrameCount()    { return validFrameCount; }
uint32_t      LidarSensor::getChecksumErrorCount() { return checksumErrorCount; }
uint32_t      LidarSensor::getLowStrengthCount()   { return lowStrengthCount; }
uint32_t      LidarSensor::getDroppedByteCount()   { return droppedByteCount; }
bool          LidarSensor::isValid()               { return distanceCM != LIDAR_INVALID_DISTANCE; }
