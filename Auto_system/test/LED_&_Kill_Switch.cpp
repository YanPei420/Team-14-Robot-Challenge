#include <Arduino.h>
#include "config.h"
#include <KillSwitch.h>

// ======================================================
// KILL SWITCH
// ======================================================

KillSwitch killSwitch(
    KILL_SWITCH_PIN
);

// ======================================================
// LED FLASH CONTROL
// ======================================================

unsigned long lastFlashTime = 0;
bool redLedState = false;

const unsigned long RED_FLASH_INTERVAL_MS = 300;

void setup()
{
    Serial.begin(115200);

    killSwitch.begin();

    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);

    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);

    Serial.println("KILL SWITCH + LED SYSTEM READY");
}

void loop()
{
    killSwitch.update();

    if (killSwitch.isTriggered())
    {
        Serial.println("EMERGENCY STOP");

        digitalWrite(LED_GREEN_PIN, LOW);

        if (millis() - lastFlashTime >= RED_FLASH_INTERVAL_MS)
        {
            lastFlashTime = millis();
            redLedState = !redLedState;
            digitalWrite(LED_RED_PIN, redLedState);
        }

        return;
    }

    Serial.println("SYSTEM OK");

    redLedState = true;
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);

    delay(100);
}