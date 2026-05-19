#include "WiFiHandler.h"

#include <stdio.h>
#include <string.h>

WiFiHandler* WiFiHandler::activeInstance = nullptr;

WiFiHandler::WiFiHandler(
    const char* wifiSSID,
    const char* wifiPassword,
    const char* mqttBrokerHost,
    uint16_t mqttBrokerPort,
    const char* teamGroupId,
    const char* robotBoardId
)
{
    ssid = wifiSSID;
    password = wifiPassword;
    brokerHost = mqttBrokerHost;
    brokerPort = mqttBrokerPort;
    groupId = teamGroupId;
    boardId = robotBoardId;

    stopTriggered = false;
    safetyEnabled = false;
    emergencyActive = false;
    disableActive = false;
    hasHeartbeat = false;
    lastHeartbeatMs = 0;
    lastRegisterMs = 0;

    lastReason[0] = '\0';
    lastTagId[0] = '\0';
    lastMessage[0] = '\0';
}

void WiFiHandler::begin()
{
    if (!credentialsConfigured())
    {
        Serial.println("WiFi/MQTT credentials are incomplete, skip connection");
        return;
    }

    activeInstance = this;
    resetMessageState();
    messenger.onMessage(handleIncomingMessage);

    const bool started =
        messenger.begin(
            ssid,
            password,
            brokerHost,
            brokerPort,
            groupId,
            boardId
        );

    Serial.print("MiniMessenger begin: ");
    Serial.println(started ? "connected" : "connecting");
}

void WiFiHandler::update()
{
    messenger.loop();
    updateSafetyState();

    if (
        messenger.isConnected()
        &&
        (
            lastRegisterMs == 0
            ||
            (millis() - lastRegisterMs) >= WIFI_REGISTER_INTERVAL_MS
        )
    )
    {
        if (sendRegister())
        {
            lastRegisterMs = millis();
        }
    }
}

bool WiFiHandler::isConnected()
{
    return messenger.isConnected();
}

IPAddress WiFiHandler::getIP()
{
    return WiFi.localIP();
}

bool WiFiHandler::isSafetyEnabled() const
{
    return safetyEnabled;
}

bool WiFiHandler::isStopTriggered() const
{
    return stopTriggered;
}

void WiFiHandler::clearStopTriggered()
{
    stopTriggered = false;
}

bool WiFiHandler::isEmergencyActive() const
{
    return emergencyActive;
}

bool WiFiHandler::isDisableActive() const
{
    return disableActive;
}

bool WiFiHandler::hasHeartbeatTimedOut() const
{
    if (!hasHeartbeat)
    {
        return true;
    }

    return (millis() - lastHeartbeatMs) > WIFI_HEARTBEAT_TIMEOUT_MS;
}

const char* WiFiHandler::getBoardId() const
{
    return boardId;
}

const char* WiFiHandler::getGroupId() const
{
    return groupId;
}

const char* WiFiHandler::getLastReason() const
{
    return lastReason;
}

const char* WiFiHandler::getLastTagId() const
{
    return lastTagId;
}

const char* WiFiHandler::getLastMessage() const
{
    return lastMessage;
}

bool WiFiHandler::sendRegister()
{
    char payload[96];

    snprintf(
        payload,
        sizeof(payload),
        "type=register team_id=%s board_id=%s",
        groupId,
        boardId
    );

    return sendServerMessage(payload);
}

bool WiFiHandler::sendIsFertile(const char* tagId)
{
    if (tagId == nullptr || tagId[0] == '\0')
    {
        return false;
    }

    char payload[128];

    snprintf(
        payload,
        sizeof(payload),
        "type=isFertile team_id=%s board_id=%s tag_id=%s",
        groupId,
        boardId,
        tagId
    );

    return sendServerMessage(payload);
}

bool WiFiHandler::sendSeedPlanted(const char* tagId)
{
    if (tagId == nullptr || tagId[0] == '\0')
    {
        return false;
    }

    char payload[128];

    snprintf(
        payload,
        sizeof(payload),
        "type=seedPlanted team_id=%s board_id=%s tag_id=%s",
        groupId,
        boardId,
        tagId
    );

    return sendServerMessage(payload);
}

bool WiFiHandler::sendOpenAirlockA()
{
    char payload[96];

    snprintf(
        payload,
        sizeof(payload),
        "type=openAirlockA team_id=%s board_id=%s",
        groupId,
        boardId
    );

    return sendServerMessage(payload);
}

bool WiFiHandler::sendOpenAirlockB()
{
    char payload[96];

    snprintf(
        payload,
        sizeof(payload),
        "type=openAirlockB team_id=%s board_id=%s",
        groupId,
        boardId
    );

    return sendServerMessage(payload);
}

bool WiFiHandler::sendToBoard(
    const char* targetBoardId,
    const char* payload
)
{
    return messenger.sendToBoard(
        targetBoardId,
        payload
    );
}

bool WiFiHandler::sendToGroup(const char* payload)
{
    return messenger.sendToGroup(payload);
}

void WiFiHandler::handleIncomingMessage(
    const MessageMetadata& metadata,
    const uint8_t* payload,
    size_t length
)
{
    if (activeInstance != nullptr)
    {
        activeInstance->onMessage(
            metadata,
            payload,
            length
        );
    }
}

void WiFiHandler::onMessage(
    const MessageMetadata& metadata,
    const uint8_t* payload,
    size_t length
)
{
    (void) metadata;

    if (payload == nullptr || length == 0)
    {
        return;
    }

    const size_t copyLength =
        (length < MiniMessenger::kMaxPayloadSize)
        ?
        length
        :
        MiniMessenger::kMaxPayloadSize;

    memcpy(
        lastMessage,
        payload,
        copyLength
    );

    lastMessage[copyLength] = '\0';

    if (containsToken(lastMessage, "type=heartbeat"))
    {
        hasHeartbeat = true;
        lastHeartbeatMs = millis();

        if (containsToken(lastMessage, "enable=1"))
        {
            safetyEnabled = true;
        }
        else if (containsToken(lastMessage, "enable=0"))
        {
            safetyEnabled = false;
            stopTriggered = true;
        }
    }

    if (containsToken(lastMessage, "type=emergency"))
    {
        emergencyActive =
            containsToken(lastMessage, "enabled=true");

        if (emergencyActive)
        {
            stopTriggered = true;
        }
    }

    if (containsToken(lastMessage, "type=disable"))
    {
        disableActive =
            containsToken(lastMessage, "enabled=false");

        if (disableActive)
        {
            stopTriggered = true;
        }

        extractValue(
            lastMessage,
            "reason=",
            lastReason,
            sizeof(lastReason)
        );
    }

    if (
        containsToken(lastMessage, "type=isFertileReply")
        ||
        containsToken(lastMessage, "type=seedPlanted")
    )
    {
        extractValue(
            lastMessage,
            "tag_id=",
            lastTagId,
            sizeof(lastTagId)
        );
    }

    updateSafetyState();
}

void WiFiHandler::resetMessageState()
{
    stopTriggered = false;
    safetyEnabled = false;
    emergencyActive = false;
    disableActive = false;
    hasHeartbeat = false;
    lastHeartbeatMs = 0;
    lastRegisterMs = 0;
    lastReason[0] = '\0';
    lastTagId[0] = '\0';
    lastMessage[0] = '\0';
}

void WiFiHandler::updateSafetyState()
{
    if (hasHeartbeatTimedOut())
    {
        safetyEnabled = false;
        stopTriggered = true;
    }

    if (emergencyActive || disableActive)
    {
        safetyEnabled = false;
        stopTriggered = true;
    }
}

bool WiFiHandler::containsToken(
    const char* text,
    const char* token
) const
{
    if (text == nullptr || token == nullptr)
    {
        return false;
    }

    return strstr(text, token) != nullptr;
}

bool WiFiHandler::extractValue(
    const char* text,
    const char* key,
    char* output,
    size_t outputSize
) const
{
    if (
        text == nullptr
        ||
        key == nullptr
        ||
        output == nullptr
        ||
        outputSize == 0
    )
    {
        return false;
    }

    const char* start = strstr(text, key);

    if (start == nullptr)
    {
        output[0] = '\0';
        return false;
    }

    start += strlen(key);

    size_t index = 0;

    while (
        start[index] != '\0'
        &&
        start[index] != ' '
        &&
        index < (outputSize - 1)
    )
    {
        output[index] = start[index];
        ++index;
    }

    output[index] = '\0';
    return index > 0;
}

bool WiFiHandler::credentialsConfigured() const
{
    return
        ssid != nullptr
        &&
        ssid[0] != '\0'
        &&
        password != nullptr
        &&
        brokerHost != nullptr
        &&
        brokerHost[0] != '\0'
        &&
        groupId != nullptr
        &&
        groupId[0] != '\0'
        &&
        boardId != nullptr
        &&
        boardId[0] != '\0';
}

bool WiFiHandler::sendServerMessage(const char* payload)
{
    return sendToBoard(
        SERVER_BOARD_ID,
        payload
    );
}
