#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------
// 传感器 I2C 地址与关键寄存器定义
// ---------------------------------------------------------
// 7-bit I2C 从机地址 (0x80 右移一位)
#define SENSOR_ADDR 0x40 

// 距离数据寄存器
#define DISTANCE_REG_HIGH 0x5E
#define DISTANCE_REG_LOW  0x5F

// 状态控制寄存器 (用于软件强制唤醒)
#define STATE_CTRL_REG 0xE8

// ---------------------------------------------------------
// 硬件引脚定义
// ---------------------------------------------------------
#define GPIO1_pin 22
#define ANALOG_pin A0

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Arduino Giga 支持 12-bit ADC 解析度 (0-4095)
  analogReadResolution(12); 

  // ==========================================
  // 1. 硬件级唤醒
  // ==========================================
  // 如果你已经把传感器的 GPIO1 直接插到了 3.3V 上，这两行其实可以不写。
  // 但保留在这里可以兼容你插回 Pin 22 的情况。
  pinMode(GPIO1_pin, OUTPUT);
  digitalWrite(GPIO1_pin, HIGH);

  delay(50); // 给硬件一点上电反应时间

  // ==========================================
  // 2. 软件级急救：I2C 强制唤醒 (破除待机锁死)
  // ==========================================
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(STATE_CTRL_REG); // 指向 Active/Stand-by 控制寄存器
  Wire.write(0x00);           // 写入 0x00 强制进入 Active (工作) 状态
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("Force Wake-up Command Sent Successfully!");
  } else {
    Serial.println("Warning: I2C Communication Error during Wake-up!");
  }

  // 给传感器一点点时间点亮红外 LED 并进行内部校准
  delay(100); 

  Serial.println("GP2Y0E03 Ultimate Test START...");
  Serial.println("--------------------------------");
}

void loop() {
  // ======================================
  // 1. 读取 I2C (Digital) 距离数据
  // ======================================
  uint8_t high_Byte = 0;
  uint8_t low_Byte = 0;
  float distance_i2c = -1.0;

  // 读取 Distance[11:4] (高位)
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(DISTANCE_REG_HIGH);
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom((uint8_t)SENSOR_ADDR, (uint8_t)1);
    if (Wire.available()) {
      high_Byte = Wire.read();
    }
  }

  // 读取 Distance[3:0] (低位)
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(DISTANCE_REG_LOW);
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom((uint8_t)SENSOR_ADDR, (uint8_t)1);
    if (Wire.available()) {
      low_Byte = Wire.read();
    }
  }

  // 将高低位组合并换算为厘米 (cm)
  uint16_t raw_i2c = (high_Byte * 16) + low_Byte;
  distance_i2c = (float)raw_i2c / 16.0 / 4.0;

  // ======================================
  // 2. 读取 Analog (模拟) 电压数据
  // ======================================
  int raw_analog = analogRead(ANALOG_pin);
  // 基于 3.3V 参考电压和 12-bit 解析度 (4095) 计算实际电压
  float voltage = (float)raw_analog * 3.3 / 4095.0;

  // ======================================
  // 3. 串口数据输出与错误研判
  // ======================================
  Serial.print("[I2C] Distance: ");
  // 加入 Error Judgment 拦截：当读数为 64cm 时，说明未测到有效信号
  if (distance_i2c >= 64.0) {
    Serial.print("Out of Range (>64cm)    ");
  } else {
    Serial.print(distance_i2c);
    Serial.print(" cm                 ");
  }

  Serial.print("| [Analog] Voltage: ");
  Serial.print(voltage, 3); // 保留三位小数，方便观察微小跳动
  Serial.println(" V");

  // 延时 100 毫秒，每秒输出 10 次
  delay(100);
}