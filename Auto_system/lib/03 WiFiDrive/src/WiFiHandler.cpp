#include "WiFiHandler.h"
#include "WiFiHandlerConfig.h"

WiFiHandler::WiFiHandler(
    const char* wifiSSID,
    const char* wifiPassword,
    uint16_t udpPort
)
{
    ssid = wifiSSID;
    password = wifiPassword;
    port = udpPort;
    stopTriggered = false;
}

void WiFiHandler::begin()
{
    WiFi.begin(
        ssid,
        password
    );

    while (
        WiFi.status()
        !=
        WL_CONNECTED
    )
    {
        delay(500);

        Serial.print(".");
    }

    Serial.println();

    Serial.println(
        "WIFI CONNECTED"
    );

    Serial.print(
        "IP: "
    );

    Serial.println(
        WiFi.localIP()
    );

    udp.begin(port);

    Serial.print(
        "UDP LISTENING ON PORT "
    );

    Serial.println(port);
}

void WiFiHandler::update()
{
    int packetSize =
        udp.parsePacket();

    if (packetSize)
    {
        int len =
            udp.read(
                incomingPacket,
                UDP_BUFFER_SIZE
            );

        if (len > 0)
        {
            incomingPacket[len] = '\0';
        }

        Serial.print(
            "UDP: "
        );

        Serial.println(
            incomingPacket
        );

        if (
            strcmp(
                incomingPacket,
                UDP_STOP_COMMAND
            )
            ==
            0
        )
        {
            stopTriggered = true;
        }
    }
}

bool WiFiHandler::isStopTriggered()
{
    return stopTriggered;
}

IPAddress WiFiHandler::getIP()
{
    return WiFi.localIP();
}