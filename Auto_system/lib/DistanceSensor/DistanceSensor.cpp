#include "DistanceSensor.h"

// 寄存器定义
#define DIST_REG_HIGH 0x5E
#define DIST_REG_LOW  0x5F
#define STATE_CTRL_REG 0xE8

DistanceSensor::DistanceSensor(uint8_t addr, int gpioPin, int analogPin) 
    : _addr(addr), _gpioPin(gpioPin), _analogPin(analogPin) {}

void DistanceSensor::begin() {
    // 设置 GPIO1 为高电平以使能传感器
    pinMode(_gpioPin, OUTPUT);
    digitalWrite(_gpioPin, HIGH);
    
    // 设置 Giga R1 的 ADC 分辨率为 12位 (0-4095)
    analogReadResolution(12);
    
    delay(50);
    wakeUp();
    delay(100);
}

bool DistanceSensor::wakeUp() {
    // 写入状态控制寄存器，设置传感器为活动状态 (0x00)
    Wire.beginTransmission(_addr);
    Wire.write(STATE_CTRL_REG);
    Wire.write(0x00); 
    return (Wire.endTransmission() == 0);
}

float DistanceSensor::readDistanceI2C() {
    uint8_t high_Byte = 0;
    uint8_t low_Byte = 0;

    // 读取高位字节
    Wire.beginTransmission(_addr);
    Wire.write(DIST_REG_HIGH);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
        if (Wire.available()) {
            high_Byte = Wire.read();
        }
    }

    // 读取低位字节
    Wire.beginTransmission(_addr);
    Wire.write(DIST_REG_LOW);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
        if (Wire.available()) {
            low_Byte = Wire.read();
        }
    }

    /**
     * 距离计算公式:
     * 1. 组合高 8 位和低 4 位得到 raw 数值
     * 2. 这里的计算方式参考了规格说明书的偏移处理
     */
    uint16_t raw_i2c = (high_Byte * 16) + low_Byte;
    return (float)raw_i2c / 16.0 / 4.0;
}

float DistanceSensor::readVoltage() {
    int raw = readRawAnalog();
    // 基于 3.3V 参考电压和 12位分辨率计算
    return (float)raw * 3.3 / 4095.0;
}

int DistanceSensor::readRawAnalog() {
    return analogRead(_analogPin);
}
