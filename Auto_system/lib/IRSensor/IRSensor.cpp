#include "IRSensor.h"

IRSensor::IRSensor(int pin) : _pin(pin) {}

void IRSensor::begin() {
    // 设置引脚为输入模式
    pinMode(_pin, INPUT);
}

int IRSensor::readDistance() {
    // 返回模拟引脚的原始读取值
    return analogRead(_pin);
}
