#include "LidarSensor.h"

LidarSensor::LidarSensor(HardwareSerial& serialPort)
    : serial(serialPort)
{
    distanceCM   = LIDAR_INVALID_DISTANCE;
    strength     = 0;
    temperature  = 0.0f;
    lastUpdateMs = 0;
    validFrameCount = 0;
    checksumErrorCount = 0;
    lowStrengthCount = 0;
    droppedByteCount = 0;
    frameIndex = 0;
}

void LidarSensor::begin()
{
    serial.begin(LIDAR_BAUD_RATE);

    if (LIDAR_DEBUG)
    {
        Serial.print("[Lidar] UART begin at ");
        Serial.println(LIDAR_BAUD_RATE);
    }
}

bool LidarSensor::update()
{
    bool receivedValidFrame = false;

    while (serial.available() > 0)
    {
        const uint8_t byte = static_cast<uint8_t>(serial.read());

        if (frameIndex == 0)
        {
            if (byte == 0x59)
            {
                frameBuffer[frameIndex++] = byte;
            }
            else
            {
                ++droppedByteCount;
            }
            continue;
        }

        if (frameIndex == 1)
        {
            if (byte == 0x59)
            {
                frameBuffer[frameIndex++] = byte;
            }
            else
            {
                frameIndex = 0;
                ++droppedByteCount;
            }
            continue;
        }

        frameBuffer[frameIndex++] = byte;

        if (frameIndex < 9)
        {
            continue;
        }

        frameIndex = 0;

        uint8_t checksum = 0;
        for (uint8_t i = 0; i < 8; i++) checksum += frameBuffer[i];
        if (checksum != frameBuffer[8])
        {
            ++checksumErrorCount;
            if (LIDAR_DEBUG) Serial.println("[Lidar] checksum error");
            continue;
        }

        if (parseFrame(frameBuffer))
        {
            ++validFrameCount;
            receivedValidFrame = true;
        }
    }

    return receivedValidFrame;
}

bool LidarSensor::parseFrame(uint8_t* buf)
{
    int16_t  dist = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    uint16_t amp  = (uint16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    uint16_t rawTemp = (uint16_t)(buf[6] | ((uint16_t)buf[7] << 8));
    float    temp = rawTemp / 8.0f - 256.0f;

    if (amp < LIDAR_MIN_STRENGTH)
    {
        ++lowStrengthCount;
        if (LIDAR_DEBUG)
        {
            Serial.print("[Lidar] low strength: ");
            Serial.println(amp);
        }
        return false;
    }

    distanceCM   = dist;
    strength     = amp;
    temperature  = temp;
    lastUpdateMs = millis();

    if (LIDAR_DEBUG)
    {
        Serial.print("[Lidar] dist=");
        Serial.print(distanceCM);
        Serial.print("cm  str=");
        Serial.print(strength);
        Serial.print("  temp=");
        Serial.println(temperature);
    }

    return true;
}

int16_t       LidarSensor::getDistanceCM()   { return distanceCM; }
uint16_t      LidarSensor::getStrength()     { return strength; }
float         LidarSensor::getTemperature()  { return temperature; }
unsigned long LidarSensor::getLastUpdateMs() { return lastUpdateMs; }
int           LidarSensor::getBytesAvailable() { return serial.available(); }
uint32_t      LidarSensor::getValidFrameCount() { return validFrameCount; }
uint32_t      LidarSensor::getChecksumErrorCount() { return checksumErrorCount; }
uint32_t      LidarSensor::getLowStrengthCount() { return lowStrengthCount; }
uint32_t      LidarSensor::getDroppedByteCount() { return droppedByteCount; }
bool          LidarSensor::isValid()         { return distanceCM != LIDAR_INVALID_DISTANCE; }
