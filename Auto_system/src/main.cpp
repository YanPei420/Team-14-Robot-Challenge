#include <Arduino.h>
#include "config.h"
#include <KillSwitch.h>
#include <WiFiHandler.h>

// ======================================================
// KILL SWITCHS
// ======================================================

KillSwitch killSwitch(
    KILL_SWITCH_PIN
);

WiFiHandler wifiHandler(
    WIFI_SSID,
    WIFI_PASSWORD,
    WIFI_UDP_PORT
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
    wifiHandler.begin();

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);

    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);

    while (!Serial)
    {
        ;
    }


    Serial.println("SYSTEM READY");
    Serial.print("IP ADDRESS: ");
    Serial.println(wifiHandler.getIP());
}

void loop()
{
    killSwitch.update();
    wifiHandler.update();

    // ======================================================
    // EMERGENCY STATE
    // ======================================================

    if (killSwitch.isTriggered() || wifiHandler.isStopTriggered())
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
