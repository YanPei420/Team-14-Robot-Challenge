#include "IRSensor.h"

IRSensor::IRSensor(
    const uint8_t* sensorPins,
    uint8_t sensorCount
)
{
    pins = sensorPins;
    count = sensorCount;

    for (uint8_t i = 0; i < count; i++)
    {
        readings[i] = false;
    }

    lastPosition = 0.0f;
}

void IRSensor::begin()
{
    for (uint8_t i = 0; i < count; i++)
    {
        pinMode(pins[i], INPUT);
    }

    if (IR_DEBUG)
    {
        Serial.println("[IR] Initialized");
    }
}

void IRSensor::update()
{
    for (uint8_t i = 0; i < count; i++)
    {
        readings[i] = sensorActive(i);
    }

    if (IR_DEBUG)
    {
        Serial.print("[IR] ");
        for (uint8_t i = 0; i < count; i++)
        {
            Serial.print(readings[i] ? "1" : "0");
        }
        Serial.print("  pos=");
        Serial.println(getLinePosition());
    }
}

bool IRSensor::isDetected(uint8_t index)
{
    if (index >= count)
    {
        return false;
    }

    return readings[index];
}

bool IRSensor::isLineDetected()
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (readings[i])
        {
            return true;
        }
    }

    return false;
}

float IRSensor::getLinePosition()
{
    // Weighted centroid: sensor positions mapped to [-1, +1]
    float weightedSum = 0.0f;
    float totalWeight = 0.0f;

    for (uint8_t i = 0; i < count; i++)
    {
        if (readings[i])
        {
            // Map index [0, count-1] to [-1.0, +1.0]
            float pos = (2.0f * i / (count - 1)) - 1.0f;
            weightedSum += pos;
            totalWeight += 1.0f;
        }
    }

    if (totalWeight > 0.0f)
    {
        lastPosition = weightedSum / totalWeight;
    }

    return lastPosition;
}

uint8_t IRSensor::getCount()
{
    return count;
}

bool IRSensor::sensorActive(uint8_t index)
{
    return (digitalRead(pins[index]) == IR_LINE_STATE);
}
