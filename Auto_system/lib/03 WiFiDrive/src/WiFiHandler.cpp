#include "WiFiHandler.h"

#include <string.h>

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
    udpStarted = false;
}

void WiFiHandler::begin()
{
    if (!credentialsConfigured())
    {
        Serial.println("WiFi credentials are empty, skip connection");
        return;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("WiFi already connected, IP: ");
        Serial.println(WiFi.localIP());

        udp.begin(port);
        udpStarted = true;
        return;
    }

    scanNetworks();

    for (uint8_t attempt = 1; attempt <= WIFI_MAX_CONNECT_ATTEMPTS; ++attempt)
    {
        Serial.print("[Attempt ");
        Serial.print(attempt);
        Serial.print("/");
        Serial.print(WIFI_MAX_CONNECT_ATTEMPTS);
        Serial.print("] Connecting to '");
        Serial.print(ssid);
        Serial.println("'...");

        WiFi.begin(
            ssid,
            password
        );

        const uint32_t startMs = millis();
        uint8_t previousStatus = 0xFF;
        uint8_t status = WiFi.status();

        while (
            status != WL_CONNECTED
            &&
            (millis() - startMs) < WIFI_CONNECT_TIMEOUT_MS
        )
        {
            delay(WIFI_RETRY_INTERVAL_MS);
            status = WiFi.status();

            if (status != previousStatus)
            {
                Serial.print("  status: ");
                Serial.print(wifiStatusString(status));
                Serial.print(" (");
                Serial.print(status);
                Serial.println(")");
                previousStatus = status;
            }
            else
            {
                Serial.print(".");
            }
        }

        Serial.println();

        if (status == WL_CONNECTED)
        {
            Serial.print("WiFi connected, IP: ");
            Serial.println(WiFi.localIP());

            udp.begin(port);
            udpStarted = true;

            Serial.print("UDP listening on port ");
            Serial.println(port);
            return;
        }

        Serial.print("  Attempt ");
        Serial.print(attempt);
        Serial.print(" failed. Final status: ");
        Serial.print(wifiStatusString(status));
        Serial.print(" (");
        Serial.print(status);
        Serial.println(")");

        if (attempt < WIFI_MAX_CONNECT_ATTEMPTS)
        {
            Serial.println("  Retrying in 2 s...");
            delay(2000);
        }
    }

    Serial.print("WiFi connection failed after ");
    Serial.print(WIFI_MAX_CONNECT_ATTEMPTS);
    Serial.println(" attempts.");
}

void WiFiHandler::update()
{
    if (!udpStarted)
    {
        return;
    }

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
            Serial.println("UDP stop command triggered");
        }
    }
}

bool WiFiHandler::isStopTriggered()
{
    return stopTriggered;
}

void WiFiHandler::clearStopTriggered()
{
    stopTriggered = false;
}

bool WiFiHandler::isConnected()
{
    return WiFi.status() == WL_CONNECTED;
}

IPAddress WiFiHandler::getIP()
{
    return WiFi.localIP();
}

bool WiFiHandler::credentialsConfigured()
{
    return ssid != nullptr && ssid[0] != '\0';
}

const char* WiFiHandler::wifiStatusString(uint8_t status)
{
    switch (status)
    {
        case WL_IDLE_STATUS:
            return "IDLE";
        case WL_NO_SSID_AVAIL:
            return "NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED:
            return "SCAN_COMPLETED";
        case WL_CONNECTED:
            return "CONNECTED";
        case WL_CONNECT_FAILED:
            return "CONNECT_FAILED";
        case WL_CONNECTION_LOST:
            return "CONNECTION_LOST";
        case WL_DISCONNECTED:
            return "DISCONNECTED";
        default:
            return "UNKNOWN";
    }
}

void WiFiHandler::scanNetworks()
{
    Serial.println("Scanning nearby WiFi networks...");

    const int networkCount = WiFi.scanNetworks();

    if (networkCount < 0)
    {
        Serial.print("WiFi scan failed, error: ");
        Serial.println(networkCount);
        return;
    }

    if (networkCount == 0)
    {
        Serial.println("No WiFi networks found.");
        return;
    }

    Serial.print("Found ");
    Serial.print(networkCount);
    Serial.println(" network(s):");

    bool targetFound = false;

    for (int i = 0; i < networkCount; ++i)
    {
        const bool isTarget =
            strcmp(
                WiFi.SSID(i),
                ssid
            )
            ==
            0;

        if (isTarget)
        {
            targetFound = true;
        }

        Serial.print("  [");
        Serial.print(i + 1);
        Serial.print("] ");
        Serial.print(WiFi.SSID(i));
        Serial.print(" RSSI: ");
        Serial.print(WiFi.RSSI(i));
        Serial.print(" dBm");

        if (isTarget)
        {
            Serial.print(" <<< TARGET");
        }

        Serial.println();
    }

    if (!targetFound)
    {
        Serial.print("WARNING: Target SSID '");
        Serial.print(ssid);
        Serial.println("' not seen in scan!");
    }
}
