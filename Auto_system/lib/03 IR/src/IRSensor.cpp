#include "IRSensor.h"

IRSensor::IRSensor(
    const uint8_t* sensorPins,
    uint8_t sensorCount
)
{
    pins = sensorPins;
    count = sensorCount > IR_SENSOR_COUNT ? IR_SENSOR_COUNT : sensorCount;

    for (uint8_t i = 0; i < count; i++)
    {
        values[i] = IR_READ_TIMEOUT_US;
    }
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
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], HIGH);
    }

    delayMicroseconds(IR_CHARGE_TIME_US);

    for (uint8_t i = 0; i < count; i++)
    {
        pinMode(pins[i], INPUT);
        values[i] = IR_READ_TIMEOUT_US;
    }

    const unsigned long startTime = micros();

    while (micros() - startTime < IR_READ_TIMEOUT_US)
    {
        const uint16_t elapsedUs =
            static_cast<uint16_t>(micros() - startTime);

        for (uint8_t i = 0; i < count; i++)
        {
            if (values[i] == IR_READ_TIMEOUT_US && digitalRead(pins[i]) == LOW)
            {
                values[i] = elapsedUs;
            }
        }
    }

    if (IR_DEBUG)
    {
        Serial.print("[IR] ");
        for (uint8_t i = 0; i < count; i++)
        {
            Serial.print(values[i]);
            Serial.print('\t');
        }
        Serial.println();
    }
}

uint16_t IRSensor::getValue(uint8_t index)
{
    if (index >= count)
    {
        return IR_READ_TIMEOUT_US;
    }

    return values[index];
}

const uint16_t* IRSensor::getValues()
{
    return values;
}

uint8_t IRSensor::getCount()
{
    return count;
}
