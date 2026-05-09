#include "ServoControl.h"

ServoControl::ServoControl(int pin) : _pin(pin) {}

void ServoControl::begin() {
    // 将引脚绑定到舵机对象
    _servo.attach(_pin);
}

void ServoControl::setAngle(int angle) {
    // 写入角度
    _servo.write(angle);
}
