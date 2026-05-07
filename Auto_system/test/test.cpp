#include <Arduino.h>
#include <Wire.h>

#define SENSOR_ADDR 0x40

#define DISTANCE_REG_HIGH 0x5E
#define DISTANCE_REG_LOW 0x5F

#define GPIO1_pin 22
#define ANALOG_pin A0

void setup() {
  Serial.begin(115200);
  Wire.begin();

  analogReadResolution(12); // GIGA gets 12-bit resolution

  pinMode(GPIO1_pin, OUTPUT);
  digitalWrite(GPIO1_pin, HIGH);

  delay(50);
  Serial.println("GP2Y0E03 distance sensor test START...");
  Serial.println("--------------------------------");
}

void loop() {
  //======================================
  // I2C (Digital) Data
  //======================================
  uint8_t high_Byte = 0;
  uint8_t low_Byte = 0;
  float distance_i2c = -1.0;

  // Read Distance 11:4
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(DISTANCE_REG_HIGH);
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom((uint8_t)SENSOR_ADDR, (uint8_t)1);
    if (Wire.available()) {
      high_Byte = Wire.read();
    }
  }

  // Read Distance 3:0
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(DISTANCE_REG_LOW);
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom((uint8_t)SENSOR_ADDR, (uint8_t)1);
    if (Wire.available()) {
      low_Byte = Wire.read();
    }
  }

  // Convert distance to cm
  uint16_t raw_i2c = (high_Byte * 16) + low_Byte;
  distance_i2c = (float)raw_i2c / 16.0 / 4.0;

  //======================================
  // Analog Data
  //======================================
  int raw_analog = analogRead(ANALOG_pin);
  float voltage = (float)raw_analog * 3.3 / 4095.0;

  //======================================
  // Output
  //======================================
  Serial.print("I2C Distance: ");
  // 加入 Error Judgment 拦截
  if (distance_i2c >= 64.0) {
    Serial.println("Out of Range (>64cm)");
  } else {
    Serial.print(distance_i2c);
    Serial.println(" cm");
  }

  Serial.print("Analog Voltage: ");
  Serial.print(voltage, 3); // 保留三位小数，方便观察动态变化
  Serial.println(" V\n");

  delay(100);
}