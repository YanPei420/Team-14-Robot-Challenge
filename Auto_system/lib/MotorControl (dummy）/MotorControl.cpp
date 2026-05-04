#include "MotorControl.h"

MotorControl::MotorControl() {}

void MotorControl::begin() {
    Wire.begin();
    _motors.reinitialization() ;
    _motors.clearResetFlag();
}

void MotorControl::setSpeed(int speed) {
    _motors.setSpeed(1, speed);
    _motors.setSpeed(2, speed);
}

void MotorControl::stop() {
    _motors.setSpeed(1, 0);
    _motors.setSpeed(2, 0);
}
