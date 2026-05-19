#include <Arduino.h>

#include "WiFiHandler.h"
#include "WiFiHandlerConfig.h"

WiFiHandler wifiHandler;

unsigned long lastStatusPrintMs = 0;
bool lastConnectedState = false;

void printWiFiTestBanner()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("WiFi Test");
    Serial.println("========================================");
    Serial.print("Target SSID: ");
    Serial.println(WIFI_SSID);
    Serial.print("MQTT broker: ");
    Serial.print(BROKER_HOST);
    Serial.print(":");
    Serial.println(BROKER_PORT);
    Serial.print("Group ID: ");
    Serial.println(GROUP_ID);
    Serial.print("Board ID: ");
    Serial.println(BOARD_ID);
    Serial.println("========================================");
}

void printConnectionStatus()
{
    const bool connected = wifiHandler.isConnected();

    if (connected != lastConnectedState)
    {
        lastConnectedState = connected;

        if (connected)
        {
            Serial.print("WiFi connected, local IP: ");
            Serial.println(wifiHandler.getIP());
        }
        else
        {
            Serial.println("WiFi disconnected");
        }
    }
}

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
    }

    delay(500);

    printWiFiTestBanner();
    wifiHandler.begin();
    printConnectionStatus();
}

void loop()
{
    wifiHandler.update();
    printConnectionStatus();

    if ((millis() - lastStatusPrintMs) >= 5000)
    {
        lastStatusPrintMs = millis();

        Serial.print("Heartbeat: Link ");
        Serial.print(wifiHandler.isConnected() ? "OK" : "NOT CONNECTED");
        Serial.print(", Safety ");
        Serial.print(wifiHandler.isSafetyEnabled() ? "ENABLED" : "DISABLED");

        if (wifiHandler.isConnected())
        {
            Serial.print(", IP: ");
            Serial.println(wifiHandler.getIP());
        }
        else
        {
            Serial.println();
        }
    }

    if (wifiHandler.isStopTriggered())
    {
        Serial.print("Test result: stop triggered");

        if (wifiHandler.getLastReason()[0] != '\0')
        {
            Serial.print(", reason=");
            Serial.print(wifiHandler.getLastReason());
        }

        Serial.println();
        wifiHandler.clearStopTriggered();
    }

    delay(50);
}
