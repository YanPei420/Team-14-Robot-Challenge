#ifndef LED_H
#define LED_H

#include <Arduino.h>

#include "LEDConfig.h"

class LED
{
private:
    uint8_t redPin;
    uint8_t greenPin;

    unsigned long redFlashIntervalMs;
    unsigned long lastFlashTime;

    bool redState;
    bool greenState;

    void writeRed(bool state);
    void writeGreen(bool state);

public:
    LED(
        uint8_t redLedPin = LED_RED_PIN,
        uint8_t greenLedPin = LED_GREEN_PIN,
        unsigned long flashIntervalMs = LED_RED_FLASH_INTERVAL_MS
    );

    void begin();

    void setRed(bool state);
    void setGreen(bool state);
    void setBoth(bool red, bool green);
    void allOff();

    void showNormal();
    void showButtonPressed();
    void showEmergency();
};

#endif
