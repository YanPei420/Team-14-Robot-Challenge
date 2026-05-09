#include <Arduino.h>
#include "config.h"
#include "LidarSensor.h"

LidarSensor lidar;

void setup() {
    Serial.begin(115200);
    lidar.begin();
    Serial.println("TF-Luna Lidar Library Test");
    Serial.println("--------------------------");
}

void loop() {
    lidar.update();
    
    if (lidar.isReliable()) {
        Serial.print("Dist: ");
        Serial.print(lidar.getDistance());
        Serial.print(" cm | Amp: ");
        Serial.print(lidar.getAmplitude());
        Serial.print(" | Temp: ");
        Serial.print(lidar.getTemperature());
        Serial.println(" C");
    } else {
        Serial.println("Signal Weak (Unreliable)");
    }
    
    delay(100);
}
