#include <Arduino.h>
#include "ServoSweep.h"

ServoSweep lidarServo(SERVO_PIN);

unsigned long lastPrintMs = 0;

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) { ; }
    Serial.println("=== Servo sweep test ===");
    lidarServo.begin();
}

void loop() {
    lidarServo.update();

    if (millis() - lastPrintMs >= SERVO_PRINT_INTERVAL_MS) {
        lastPrintMs = millis();
        Serial.print("angle: ");
        Serial.print(lidarServo.getAngle());
        Serial.println(" deg");
    }
}
