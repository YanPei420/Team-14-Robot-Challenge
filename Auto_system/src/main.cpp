#include <Arduino.h>
#include "LidarSensor.h"
#include "LidarConfig.h"

LidarSensor lidar1(LIDAR_SERIAL_LEFT);
LidarSensor lidar2(LIDAR_SERIAL_RIGHT);
LidarSensor lidar3(LIDAR_SERIAL_FRONT);

void printReading(const char* label, LidarSensor& lidar)
{
    Serial.print(label);
    Serial.print(": ");

    if (!lidar.isValid())
    {
        Serial.println("no data yet");
        return;
    }

    unsigned long age = millis() - lidar.getLastUpdateMs();
    Serial.print(lidar.getDistanceCM());
    Serial.print(" cm  (amp=");
    Serial.print(lidar.getStrength());
    Serial.print(", age=");
    Serial.print(age);
    Serial.println("ms)");
}

void setup()
{
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) { ; }

    lidar1.begin();
    lidar2.begin();
    lidar3.begin();
}

void loop()
{
    lidar1.update();
    lidar2.update();
    lidar3.update();

    static uint32_t lastPrintMs = 0;
    if (millis() - lastPrintMs >= 100)
    {
        lastPrintMs = millis();
        Serial.println("---");
        printReading("LIDAR1 (left) ", lidar1);
        printReading("LIDAR2 (right)", lidar2);
        printReading("LIDAR3 (front)", lidar3);
    }
}
