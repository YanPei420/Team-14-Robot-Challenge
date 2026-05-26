#include "LidarSensor.h"

LidarSensor::LidarSensor(
    TwoWire& wireBus,
    uint8_t sensorAddress
)
    : wire(wireBus)
{
    address = sensorAddress;

    distanceCM = LIDAR_INVALID_DISTANCE;
    strength = 0;
    temperature = 0;
}

void LidarSensor::begin()
{
    wire.begin();

    if (LIDAR_DEBUG)
    {
        Serial.print("[Lidar] begin at I2C 0x");
        Serial.println(address, HEX);
    }
}

bool LidarSensor::update()
{
    // Request 6 bytes starting from register LIDAR_REG_DIST_L
    wire.beginTransmission(address);
    wire.write(LIDAR_REG_DIST_L);

    if (wire.endTransmission(false) != 0)
    {
        if (LIDAR_DEBUG)
        {
            Serial.println("[Lidar] I2C transmit error");
        }
        return false;
    }

    uint8_t received = wire.requestFrom(
        (uint8_t)address,
        (uint8_t)6
    );

    if (received != 6)
    {
        if (LIDAR_DEBUG)
        {
            Serial.println("[Lidar] incomplete frame");
        }
        return false;
    }

    uint8_t buf[6];
    for (uint8_t i = 0; i < 6; i++)
    {
        buf[i] = wire.read();
    }

    return parseFrame(buf);
}

bool LidarSensor::parseFrame(uint8_t* buf)
{
    int16_t dist  = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    uint16_t flux = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    int16_t temp  = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));

    if (flux < LIDAR_MIN_STRENGTH)
    {
        if (LIDAR_DEBUG)
        {
            Serial.print("[Lidar] low strength: ");
            Serial.println(flux);
        }
        return false;
    }

    distanceCM = dist;
    strength   = flux;
    temperature = temp;

    if (LIDAR_DEBUG)
    {
        Serial.print("[Lidar] dist=");
        Serial.print(distanceCM);
        Serial.print("cm  str=");
        Serial.print(strength);
        Serial.print("  temp=");
        Serial.println(temperature / 100.0f);
    }

    return true;
}

int16_t LidarSensor::getDistanceCM()
{
    return distanceCM;
}

uint16_t LidarSensor::getStrength()
{
    return strength;
}

int16_t LidarSensor::getTemperature()
{
    return temperature;
}

bool LidarSensor::isValid()
{
    return (distanceCM != LIDAR_INVALID_DISTANCE);
}
