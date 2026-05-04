#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <Servo.h>

class ServoControl {
public:
    ServoControl(int pin);
    void begin();
    void setAngle(int angle);
private:
    Servo _servo;
    int _pin;
};

#endif
