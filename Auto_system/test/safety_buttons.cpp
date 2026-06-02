#include <Arduino.h>

#include "KillSwitch.h"
#include "KillSwitchConfig.h"
#include "ReviveButton.h"
#include "ReviveButtonConfig.h"

KillSwitch killSwitch;
ReviveButton reviveButton;

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t PRINT_INTERVAL_MS = 250;

bool lastKillTriggered = false;
bool lastRevivePressed = false;
uint32_t lastPrintMs = 0;

const char* levelName(uint8_t level)
{
    return level == HIGH ? "HIGH" : "LOW";
}

void printButtonStates(bool force)
{
    const uint32_t now = millis();
    if (!force && now - lastPrintMs < PRINT_INTERVAL_MS)
    {
        return;
    }

    lastPrintMs = now;

    const uint8_t rawKill = digitalRead(KILL_SWITCH_PIN);
    const uint8_t rawRevive = digitalRead(REVIVE_BUTTON_PIN);

    killSwitch.update();
    reviveButton.update();

    const bool killTriggered = killSwitch.isTriggered();
    const bool revivePressed = reviveButton.isPressed();

    Serial.print("[safety] kill pin=");
    Serial.print(KILL_SWITCH_PIN);
    Serial.print(" raw=");
    Serial.print(levelName(rawKill));
    Serial.print(" active=");
    Serial.print(levelName(KILL_SWITCH_ACTIVE_STATE));
    Serial.print(" state=");
    Serial.print(killTriggered ? "TRIGGERED" : "SAFE");

    Serial.print(" | revive pin=");
    Serial.print(REVIVE_BUTTON_PIN);
    Serial.print(" raw=");
    Serial.print(levelName(rawRevive));
    Serial.print(" active=");
    Serial.print(levelName(REVIVE_BUTTON_ACTIVE_STATE));
    Serial.print(" state=");
    Serial.println(revivePressed ? "PRESSED" : "RELEASED");

    if (killTriggered != lastKillTriggered)
    {
        Serial.print("[safety] kill changed -> ");
        Serial.println(killTriggered ? "TRIGGERED" : "SAFE");
        lastKillTriggered = killTriggered;
    }

    if (revivePressed != lastRevivePressed)
    {
        Serial.print("[safety] revive changed -> ");
        Serial.println(revivePressed ? "PRESSED" : "RELEASED");
        lastRevivePressed = revivePressed;
    }
}
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 5000)
    {
    }

    killSwitch.begin();
    reviveButton.begin();
    killSwitch.update();
    reviveButton.update();

    lastKillTriggered = killSwitch.isTriggered();
    lastRevivePressed = reviveButton.isPressed();

    Serial.println("Safety buttons connection test");
    Serial.println("Both inputs use INPUT_PULLUP.");
    Serial.println("Released/untriggered should read HIGH.");
    Serial.println("Pressed/triggered should read LOW.");
    printButtonStates(true);
}

void loop()
{
    printButtonStates(false);
}
