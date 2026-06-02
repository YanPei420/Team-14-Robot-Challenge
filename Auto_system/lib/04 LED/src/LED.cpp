#include "LED.h"

LED::LED(
    uint8_t redLedPin,
    uint8_t greenLedPin,
    unsigned long flashIntervalMs
)
{
    redPin = redLedPin;
    greenPin = greenLedPin;
    redFlashIntervalMs = flashIntervalMs;

    lastFlashTime = 0;
    redState = false;
    greenState = false;
}

void LED::begin()
{
    pinMode(
        redPin,
        OUTPUT
    );

    pinMode(
        greenPin,
        OUTPUT
    );

    showNormal();
}

void LED::setRed(bool state)
{
    writeRed(state);
}

void LED::setGreen(bool state)
{
    writeGreen(state);
}

void LED::setBoth(bool red, bool green)
{
    writeRed(red);
    writeGreen(green);
}

void LED::allOff()
{
    setBoth(
        LOW,
        LOW
    );
}

void LED::showNormal()
{
    writeGreen(LOW);

    if (millis() - lastFlashTime >= redFlashIntervalMs)
    {
        lastFlashTime = millis();
        writeRed(!redState);
    }
}

void LED::showButtonPressed()
{
    setBoth(
        LOW,
        HIGH
    );

    lastFlashTime = millis();
}

void LED::showEmergency()
{
    setBoth(
        HIGH,
        LOW
    );

    lastFlashTime = millis();
}

void LED::writeRed(bool state)
{
    redState = state;

    digitalWrite(
        redPin,
        redState ? HIGH : LOW
    );
}

void LED::writeGreen(bool state)
{
    greenState = state;

    digitalWrite(
        greenPin,
        greenState ? HIGH : LOW
    );
}
