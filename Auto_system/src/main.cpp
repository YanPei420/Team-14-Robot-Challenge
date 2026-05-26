#include <Arduino.h>
#include <Wire.h>
#include "LidarSensor.h"
// TF-Luna SDA -> Arduino SDA, TF-Luna SCL -> Arduino SCL

LidarSensor lidar(LIDAR_WIRE, LIDAR_I2C_ADDRESS);

static void i2cScan() {
  Serial.println("Scanning I2C bus...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    LIDAR_WIRE.beginTransmission(addr);
    if (LIDAR_WIRE.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("  No I2C devices found");
  Serial.print("LIDAR_I2C_ADDRESS configured: 0x");
  Serial.println(LIDAR_I2C_ADDRESS, HEX);
}

void setup() {
  Serial.begin(115200);
  // Timeout so it doesn't hang if USB is slow to connect
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) { ; }

  Serial.println("TF-Luna Lidar I2C Test");
  LIDAR_WIRE.begin();
  i2cScan();
  lidar.begin();
}

void loop() {
  if (lidar.update()) {
    Serial.print("Distance: ");
    Serial.print(lidar.getDistanceCM());
    Serial.print(" cm  Strength: ");
    Serial.println(lidar.getStrength());
  } else {
    Serial.println("Distance: UNRELIABLE (low signal or I2C error)");
  }
  delay(200);
}
