#include <Arduino.h>
#include <KillSwitch.h>
#include <LED.h>
#include <ReviveButton.h>

// ======================================================
// KILL SWITCHS
// ======================================================

KillSwitch killSwitch(
    KILL_SWITCH_PIN
);

LED statusLed;
ReviveButton reviveButton;

void setup()
{
    Serial.begin(115200);

    killSwitch.begin();
    statusLed.begin();
    reviveButton.begin();

    while (!Serial)
    {
        ;
    }


    Serial.println("SYSTEM READY");
}

void loop()
{
    killSwitch.update();
    reviveButton.update();

    // ======================================================
    // EMERGENCY STATE
    // ======================================================

    if (killSwitch.isTriggered())
    {
        Serial.println("EMERGENCY STOP");

        statusLed.showEmergency();

        return;
    }

    // ======================================================
    // BUTTON STATE
    // ======================================================

    if (reviveButton.isPressed())
    {
        Serial.println("BUTTON PRESSED");

        statusLed.showButtonPressed();

        delay(3000);
        return;
    }

    // ======================================================
    // NORMAL STATE
    // ======================================================

    Serial.println("SYSTEM OK");

    statusLed.showNormal();

    delay(100);
}
