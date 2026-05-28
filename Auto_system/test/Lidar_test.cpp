#include <Arduino.h>
#include "LidarSensor.h"

LidarSensor lidar1(LIDAR_SERIAL_1);
LidarSensor lidar2(LIDAR_SERIAL_2);
LidarSensor lidar3(LIDAR_SERIAL_3);

unsigned long lastPrintMs = 0;

static void printReading(const char* label, LidarSensor& lidar) {
    Serial.print(label);
    Serial.print(": ");
    if (!lidar.isValid()) {
        Serial.print("no valid data yet");
        Serial.print("  avail=");
        Serial.print(lidar.getBytesAvailable());
        Serial.print(" frames=");
        Serial.print(lidar.getValidFrameCount());
        Serial.print(" checksum=");
        Serial.print(lidar.getChecksumErrorCount());
        Serial.print(" weak=");
        Serial.print(lidar.getLowStrengthCount());
        Serial.print(" dropped=");
        Serial.println(lidar.getDroppedByteCount());
        return;
    }
    unsigned long age = millis() - lidar.getLastUpdateMs();
    Serial.print(lidar.getDistanceCM());
    Serial.print(" cm  (amp=");
    Serial.print(lidar.getStrength());
    Serial.print(", age=");
    Serial.print(age);
    Serial.print("ms, avail=");
    Serial.print(lidar.getBytesAvailable());
    Serial.print(", frames=");
    Serial.print(lidar.getValidFrameCount());
    Serial.print(", checksum=");
    Serial.print(lidar.getChecksumErrorCount());
    Serial.print(", weak=");
    Serial.print(lidar.getLowStrengthCount());
    Serial.print(", dropped=");
    Serial.print(lidar.getDroppedByteCount());
    Serial.println(")");
}

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) { ; }
    Serial.println("TF-Luna x3 LIDAR");
    lidar1.begin();
    lidar2.begin();
    lidar3.begin();
}

void loop() {
    // Three independent hardware UARTs — poll all each iteration
    lidar1.update();
    lidar2.update();
    lidar3.update();

    if (millis() - lastPrintMs >= LIDAR_PRINT_INTERVAL_MS) {
        lastPrintMs = millis();
        Serial.println("---");
        printReading("LIDAR1", lidar1);
        printReading("LIDAR2", lidar2);
        printReading("LIDAR3", lidar3);
    }
}
