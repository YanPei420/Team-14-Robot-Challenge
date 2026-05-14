#include <Arduino.h>

#include "config.h"
#include "DistanceSensor.h"

// ======================================================
// SENSOR
// ======================================================

DistanceSensor sensor(
    SENSOR_I2C,
    DIST_SENSOR_ADDR
);

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
        ;
    }

    Serial.println("DISTANCE SENSOR TEST");

    sensor.begin();

    Serial.println("SENSOR READY");
}

void loop()
{
    float distance =
        sensor.readDistanceCM();

    Serial.print("Distance: ");

    Serial.print(distance);

    Serial.println(" cm");

    delay(100);
}