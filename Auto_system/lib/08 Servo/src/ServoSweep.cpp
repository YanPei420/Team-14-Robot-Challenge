#include "ServoSweep.h"

ServoSweep::ServoSweep(uint8_t servoPin)
    : pin(servoPin)
    , currentAngle(SERVO_SWEEP_MIN)
    , direction(1)
    , lastStepMs(0)
{
}

void ServoSweep::begin()
{
    servo.attach(pin);
    servo.write(currentAngle);
    lastStepMs = millis();
}

void ServoSweep::update()
{
    if (millis() - lastStepMs < SERVO_STEP_INTERVAL_MS)
        return;

    lastStepMs = millis();

    currentAngle += direction * SERVO_SWEEP_STEP;

    if (currentAngle >= SERVO_SWEEP_MAX) {
        currentAngle = SERVO_SWEEP_MAX;
        direction = -1;
    } else if (currentAngle <= SERVO_SWEEP_MIN) {
        currentAngle = SERVO_SWEEP_MIN;
        direction = 1;
    }

    servo.write(currentAngle);

    if (SERVO_DEBUG) {
        Serial.print("[Servo] angle=");
        Serial.println(currentAngle);
    }
}

int ServoSweep::getAngle() const
{
    return currentAngle;
}
