#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "DistanceSensor.h"

DistanceSensor distSensor(ADDR_DIST_SENSOR, PIN_DIST_SENSOR_GPIO1, PIN_DIST_SENSOR_ANALOG);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  distSensor.begin();

  Serial.println("GP2Y0E03 Ultimate Test START...");
  Serial.println("--------------------------------");
}

void loop() {
  float distance_i2c = distSensor.readDistanceI2C();
  float voltage = distSensor.readVoltage();

  Serial.print("[I2C] Distance: ");
  if (distance_i2c >= 64.0) {
    Serial.print("Out of Range (>64cm)    ");
  } else {
    Serial.print(distance_i2c);
    Serial.print(" cm                 ");
  }

  Serial.print("| [Analog] Voltage: ");
  Serial.print(voltage, 3);
  Serial.println(" V");

  delay(100);
}
