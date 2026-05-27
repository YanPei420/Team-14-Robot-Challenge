#include "LidarSensor.h"

LidarSensor::LidarSensor(HardwareSerial& serialPort)
    : serial(serialPort)
{
    distanceCM   = LIDAR_INVALID_DISTANCE;
    strength     = 0;
    temperature  = 0.0f;
    lastUpdateMs = 0;
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
    if (serial.available() < 9)
        return false;

    // Scan for frame header 0x59 0x59
    while (serial.available() >= 9)
    {
        if (serial.peek() != 0x59)
        {
            serial.read(); // discard misaligned byte
            continue;
        }

        uint8_t buf[9];
        serial.readBytes(buf, 9);

        if (buf[0] != 0x59 || buf[1] != 0x59)
            continue;

        // Verify checksum (sum of bytes 0-7, take low byte)
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < 8; i++) checksum += buf[i];
        if (checksum != buf[8])
        {
            if (LIDAR_DEBUG) Serial.println("[Lidar] checksum error");
            continue;
        }

        return parseFrame(buf);
    }

    return false;
}

bool LidarSensor::parseFrame(uint8_t* buf)
{
    int16_t  dist = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    uint16_t amp  = (uint16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    float    temp = (buf[6] | ((uint16_t)buf[7] << 8)) / 100.0f;

    if (amp < LIDAR_MIN_STRENGTH)
    {
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
bool          LidarSensor::isValid()         { return distanceCM != LIDAR_INVALID_DISTANCE; }
