#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#include "WiFiHandlerConfig.h"

class WiFiHandler
{
private:
    const char* ssid;
    const char* password;

    uint16_t port;

    WiFiUDP udp;

    char incomingPacket[UDP_BUFFER_SIZE + 1];

    bool stopTriggered;
    bool udpStarted;

    bool credentialsConfigured();
    const char* wifiStatusString(uint8_t status);
    void scanNetworks();

public:
    WiFiHandler(
        const char* wifiSSID,
        const char* wifiPassword,
        uint16_t udpPort
    );

    void begin();

    void update();

    bool isStopTriggered();
    void clearStopTriggered();
    bool isConnected();

    IPAddress getIP();
};

#endif
