#include <Arduino.h>

#include "IRConfig.h"
#include "IRSensor.h"

IRSensor irSensors(IR_PINS, IR_SENSOR_COUNT);

namespace
{
constexpr unsigned long kBootDelayMs = 5000;
constexpr unsigned long kPrintDelayMs = 250;

void printSensorHeader()
{
    Serial.println("SYSTEM BOOT: Library Diagnostic Started");
    Serial.println("Emitters ON. Reading pins through IRSensor library. Wave white paper.");

    for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++)
    {
        Serial.print('P');
        Serial.print(IR_PINS[i]);

        if (i + 1 < IR_SENSOR_COUNT)
        {
            Serial.print('\t');
        }
    }

    Serial.println();

    for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++)
    {
        Serial.print("--------");

        if (i + 1 < IR_SENSOR_COUNT)
        {
            Serial.print('\t');
        }
    }

    Serial.println();
}

void printIrReadings()
{
    for (uint8_t i = 0; i < irSensors.getCount(); i++)
    {
        Serial.print(irSensors.getValue(i));

        if (i + 1 < irSensors.getCount())
        {
            Serial.print('\t');
        }
    }

    Serial.println();
}
}

void setup()
{
    delay(kBootDelayMs);

    Serial.begin(9600);

    while (!Serial)
    {
        ;
    }

    irSensors.begin();
    printSensorHeader();
}

void loop()
{
    irSensors.update();
    printIrReadings();

    delay(kPrintDelayMs);
}
