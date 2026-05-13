#include "KillSwitch.h"
#include "KillSwitchConfig.h"

KillSwitch::KillSwitch(
    uint8_t switchPin
)
{
    pin = switchPin;

    triggered = false;
}

void KillSwitch::begin()
{
    pinMode(
        pin,
        INPUT_PULLUP
    );
}

void KillSwitch::update()
{
    triggered =
    (
        digitalRead(pin)
        ==
        KILL_SWITCH_ACTIVE_STATE
    );
}

bool KillSwitch::isTriggered()
{
    return triggered;
}

bool KillSwitch::isSafe()
{
    return !triggered;
}