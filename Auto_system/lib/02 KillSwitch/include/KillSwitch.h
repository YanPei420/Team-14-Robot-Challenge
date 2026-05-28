#ifndef KILL_SWITCH_H
#define KILL_SWITCH_H

#include <Arduino.h>

#include "KillSwitchConfig.h"

class KillSwitch
{
private:
    uint8_t pin;
    bool triggered;

public:
    KillSwitch(
        uint8_t buttonPin = KILL_SWITCH_PIN
    );

    void begin();
    void update();

    bool isTriggered();

    bool isSafe();
};

#endif
