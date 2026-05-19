#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <Arduino.h>

#include <MiniMessenger.h>
#include <WiFi.h>

#include "WiFiHandlerConfig.h"

class WiFiHandler
{
private:
    static WiFiHandler* activeInstance;

    const char* ssid;
    const char* password;
    const char* brokerHost;
    uint16_t brokerPort;
    const char* groupId;
    const char* boardId;

    MiniMessenger messenger;

    bool stopTriggered;
    bool safetyEnabled;
    bool emergencyActive;
    bool disableActive;
    bool hasHeartbeat;
    unsigned long lastHeartbeatMs;
    unsigned long lastRegisterMs;

    char lastReason[32];
    char lastTagId[32];
    char lastMessage[MiniMessenger::kMaxPayloadSize + 1];

    static void handleIncomingMessage(
        const MessageMetadata& metadata,
        const uint8_t* payload,
        size_t length
    );

    void onMessage(
        const MessageMetadata& metadata,
        const uint8_t* payload,
        size_t length
    );

    void resetMessageState();
    void updateSafetyState();
    bool containsToken(const char* text, const char* token) const;
    bool extractValue(
        const char* text,
        const char* key,
        char* output,
        size_t outputSize
    ) const;
    bool credentialsConfigured() const;
    bool sendServerMessage(const char* payload);

public:
    WiFiHandler(
        const char* wifiSSID = WIFI_SSID,
        const char* wifiPassword = WIFI_PASSWORD,
        const char* mqttBrokerHost = BROKER_HOST,
        uint16_t mqttBrokerPort = BROKER_PORT,
        const char* teamGroupId = GROUP_ID,
        const char* robotBoardId = BOARD_ID
    );

    void begin();
    void update();

    bool isConnected();
    IPAddress getIP();

    bool isSafetyEnabled() const;
    bool isStopTriggered() const;
    void clearStopTriggered();
    bool isEmergencyActive() const;
    bool isDisableActive() const;
    bool hasHeartbeatTimedOut() const;

    const char* getBoardId() const;
    const char* getGroupId() const;
    const char* getLastReason() const;
    const char* getLastTagId() const;
    const char* getLastMessage() const;

    bool sendRegister();
    bool sendIsFertile(const char* tagId);
    bool sendSeedPlanted(const char* tagId);
    bool sendOpenAirlockA();
    bool sendOpenAirlockB();
    bool sendToBoard(const char* targetBoardId, const char* payload);
    bool sendToGroup(const char* payload);
};

#endif
