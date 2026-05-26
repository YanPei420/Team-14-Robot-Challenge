#include <Arduino.h>
#include "LidarSensor.h"
// TF-Luna TX -> Arduino RX1 (pin 0), TF-Luna RX -> Arduino TX1 (pin 1)

LidarSensor lidar(LIDAR_SERIAL);

void setup() {
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) { ; }
  Serial.println("TF-Luna Lidar UART Test");
  lidar.begin();
}

void loop() {
  if (lidar.update()) {
    Serial.print("Distance: ");
    Serial.print(lidar.getDistanceCM());
    Serial.println(" cm ");
  }
}
