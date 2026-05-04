#include "IRSensor.h"

IRSensor::IRSensor(int pin) : _pin(pin) {}

void IRSensor::begin() {
    pinMode(_pin, INPUT);
}

int IRSensor::readDistance() {
    return analogRead(_pin);
}
