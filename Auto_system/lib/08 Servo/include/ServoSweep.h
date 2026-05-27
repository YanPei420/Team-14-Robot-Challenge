#ifndef SERVO_SWEEP_H
#define SERVO_SWEEP_H

#include <Arduino.h>
#include <Servo.h>
#include "ServoConfig.h"

class ServoSweep
{
private:
    Servo         servo;
    uint8_t       pin;
    int           currentAngle;
    int           direction;       // +1 or -1
    unsigned long lastStepMs;

public:
    ServoSweep(uint8_t servoPin = SERVO_PIN);

    void begin();

    // Call every loop — advances one step when the interval has elapsed.
    void update();

    int getAngle() const;
};

#endif
