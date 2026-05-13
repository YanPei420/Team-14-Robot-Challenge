#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiUdp.h>

class WiFiHandler
{
private:
    const char* ssid;
    const char* password;

    uint16_t port;

    WiFiUDP udp;

    char incomingPacket[255];

    bool stopTriggered;

public:
    WiFiHandler(
        const char* wifiSSID,
        const char* wifiPassword,
        uint16_t udpPort
    );

    void begin();

    void update();

    bool isStopTriggered();

    IPAddress getIP();
};

#endif