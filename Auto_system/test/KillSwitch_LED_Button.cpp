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

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);

    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);

    Serial.println("KILL SWITCH + LED SYSTEM READY");
}

void loop()
{
    killSwitch.update();

    // ======================================================
    // EMERGENCY STATE
    // ======================================================

    if (killSwitch.isTriggered())
    {
        Serial.println("EMERGENCY STOP");

        digitalWrite(GREEN_LED_PIN, LOW);

        if (millis() - lastFlashTime >= RED_FLASH_INTERVAL_MS)
        {
            lastFlashTime = millis();
            redLedState = !redLedState;
            digitalWrite(RED_LED_PIN, redLedState);
        }

        return;
    }

    // ======================================================
    // BUTTON STATE
    // ======================================================

    if (digitalRead(BUTTON_PIN) == LOW)
    {
        Serial.println("BUTTON PRESSED");

        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, HIGH);

        delay(3000);
        return;
    }

    // ======================================================
    // NORMAL STATE
    // ======================================================

    Serial.println("SYSTEM OK");

    redLedState = true;

    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);

    delay(100);
}