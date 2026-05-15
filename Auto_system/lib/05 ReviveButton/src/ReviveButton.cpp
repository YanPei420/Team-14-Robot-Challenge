#include "ReviveButton.h"

ReviveButton::ReviveButton(
    uint8_t buttonPin
)
{
    pin = buttonPin;
    pressed = false;
}

void ReviveButton::begin()
{
    pinMode(
        pin,
        INPUT_PULLUP
    );
}

void ReviveButton::update()
{
    pressed =
    (
        digitalRead(pin)
        ==
        REVIVE_BUTTON_ACTIVE_STATE
    );
}

bool ReviveButton::isPressed()
{
    return pressed;
}

bool ReviveButton::isReleased()
{
    return !pressed;
}
