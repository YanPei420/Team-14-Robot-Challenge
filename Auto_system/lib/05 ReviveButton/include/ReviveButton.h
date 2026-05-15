#ifndef REVIVE_BUTTON_H
#define REVIVE_BUTTON_H

#include <Arduino.h>

#include "ReviveButtonConfig.h"

class ReviveButton
{
private:
    uint8_t pin;
    bool pressed;

public:
    ReviveButton(
        uint8_t buttonPin = REVIVE_BUTTON_PIN
    );

    void begin();
    void update();

    bool isPressed();
    bool isReleased();
};

#endif
