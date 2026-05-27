#include <Arduino.h>
<<<<<<< HEAD

#if defined(CORE_CM7)
#include "M7/M7_main.h"
#elif defined(CORE_CM4)
#include "M4/M4_main.h"
#endif

void setup()
{
#if defined(CORE_CM7)
    M7Core::setup();
#elif defined(CORE_CM4)
    M4Core::setup();
#endif
}

void loop()
{
#if defined(CORE_CM7)
    M7Core::loop();
#elif defined(CORE_CM4)
    M4Core::loop();
#endif
=======
#include "LidarSensor.h"

LidarSensor lidar1(LIDAR_SERIAL_1);
LidarSensor lidar2(LIDAR_SERIAL_2);
LidarSensor lidar3(LIDAR_SERIAL_3);

unsigned long lastPrintMs = 0;

static void printReading(const char* label, LidarSensor& lidar) {
    Serial.print(label);
    Serial.print(": ");
    if (!lidar.isValid()) {
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
>>>>>>> 81936fb79c3b79c9792fce6f0347655df097b684
}
