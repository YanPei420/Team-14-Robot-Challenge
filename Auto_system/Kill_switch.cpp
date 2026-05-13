#include <Arduino.h>

#define KILL_SWITCH_PIN 22

void setup()
{
    pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
    Serial.begin(115200);
    while (!Serial)
    {
        Serial.println("Waiting for serial port...")
        ; // wait for serial port to connect. Needed for native USB
    }
}

void loop()
{
    bool emergency =
        (digitalRead(KILL_SWITCH_PIN) == HIGH);

    if (emergency)
    {
        Serial.println("EMERGENCY STOP");
    }
    else
    {
        Serial.println("SYSTEM OK");
    }

    delay(100);
}