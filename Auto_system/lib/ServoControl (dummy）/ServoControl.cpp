#include "ServoControl.h"

ServoControl::ServoControl(int pin) : _pin(pin) {}

void ServoControl::begin() {
    _servo.attach(_pin);
}

void ServoControl::setAngle(int angle) {
    _servo.write(angle);
}
