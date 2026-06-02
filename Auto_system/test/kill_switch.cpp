#include <Arduino.h>

#include "KillSwitch.h"
#include "KillSwitchConfig.h"

KillSwitch killSwitch;

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t PRINT_INTERVAL_MS = 250;

bool lastTriggered = false;
uint32_t lastPrintMs = 0;

const char* levelName(uint8_t level)
{
    return level == HIGH ? "HIGH" : "LOW";
}

void printKillSwitchState(bool force)
{
    const uint32_t now = millis();
    if (!force && now - lastPrintMs < PRINT_INTERVAL_MS)
    {
        return;
    }

    lastPrintMs = now;

    const uint8_t rawLevel = digitalRead(KILL_SWITCH_PIN);
    killSwitch.update();

    const bool triggered = killSwitch.isTriggered();

    Serial.print("[kill] pin=");
    Serial.print(KILL_SWITCH_PIN);
    Serial.print(" raw=");
    Serial.print(levelName(rawLevel));
    Serial.print(" active=");
    Serial.print(levelName(KILL_SWITCH_ACTIVE_STATE));
    Serial.print(" state=");
    Serial.println(triggered ? "TRIGGERED" : "SAFE");

    if (triggered != lastTriggered)
    {
        Serial.print("[kill] changed -> ");
        Serial.println(triggered ? "TRIGGERED" : "SAFE");
        lastTriggered = triggered;
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
    lastTriggered = killSwitch.isTriggered();

    Serial.println("Kill switch connection test");
    Serial.print("Expected wiring: pin ");
    Serial.print(KILL_SWITCH_PIN);
    Serial.println(" uses INPUT_PULLUP.");
    Serial.println("SAFE should read HIGH. TRIGGERED should read LOW.");
    printKillSwitchState(true);
}

void loop()
{
    printKillSwitchState(false);
}
