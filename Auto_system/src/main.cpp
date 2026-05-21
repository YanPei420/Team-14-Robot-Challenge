#include <Arduino.h>
#include <MiniMessenger.h>
#include <string.h>

#include "MotoronDrive.h"
#include "WiFiHandlerConfig.h"

MotoronDrive robot;
MiniMessenger messenger;

constexpr int16_t DRIVE_FORWARD_SPEED = 500;

bool safetyEnabled = false;
bool stopped = true;
bool lastConnectedState = false;
unsigned long lastStatusPrintMs = 0;
unsigned long lastRegisterMs = 0;
unsigned long lastHeartbeatMs = 0;
char lastMessage[MiniMessenger::kMaxPayloadSize + 1] = "";

void driveForward()
{
    robot.forward(DRIVE_FORWARD_SPEED);
}

void stopRobot()
{
    robot.stop_all();
}

bool messageContains(const char* token)
{
    return strstr(lastMessage, token) != nullptr;
}

bool shouldForceStopFromMessage()
{
    return
        messageContains("type=stop")
        ||
        messageContains("type=emergency")
        ||
        messageContains("type=disable")
        ||
        messageContains("enable=0");
}

bool movementEnabled()
{
    return
        safetyEnabled
        &&
        lastHeartbeatMs != 0
        &&
        (millis() - lastHeartbeatMs) <= WIFI_HEARTBEAT_TIMEOUT_MS
        &&
        !shouldForceStopFromMessage();
}

void onMessage(
    const MessageMetadata& metadata,
    const uint8_t* payload,
    size_t length
)
{
    (void) metadata;

    const size_t copyLength =
        (length < MiniMessenger::kMaxPayloadSize)
        ?
        length
        :
        MiniMessenger::kMaxPayloadSize;

    memcpy(lastMessage, payload, copyLength);
    lastMessage[copyLength] = '\0';

    if (messageContains("type=heartbeat"))
    {
        lastHeartbeatMs = millis();

        if (messageContains("enable=1"))
        {
            safetyEnabled = true;
        }
        else if (messageContains("enable=0"))
        {
            safetyEnabled = false;
            Serial.println("SAFETY: heartbeat disabled");
        }
    }

    if (
        messageContains("type=stop")
        ||
        messageContains("type=emergency enabled=true")
        ||
        messageContains("type=disable enabled=false")
    )
    {
        safetyEnabled = false;
        Serial.print("SAFETY: stop command received, message=");
        Serial.println(lastMessage);
    }
}

void sendRegister()
{
    if (
        lastRegisterMs != 0
        &&
        (millis() - lastRegisterMs) < WIFI_REGISTER_INTERVAL_MS
    )
    {
        return;
    }

    char payload[96];

    snprintf(
        payload,
        sizeof(payload),
        "type=register team_id=%s board_id=%s",
        GROUP_ID,
        BOARD_ID
    );

    if (messenger.sendToBoard(SERVER_BOARD_ID, payload))
    {
        lastRegisterMs = millis();
    }
}

void printConnectionStatus()
{
    const bool connected = messenger.isConnected();

    if (connected == lastConnectedState)
    {
        return;
    }

    lastConnectedState = connected;

    if (connected)
    {
        Serial.print("WiFi connected, local IP: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("WiFi disconnected");
    }
}

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
    }

    robot.begin();
    stopRobot();

    Serial.println("WiFi enable-gated drive ready");
    Serial.print("Target SSID: ");
    Serial.println(WIFI_SSID);
    Serial.print("MQTT broker: ");
    Serial.print(BROKER_HOST);
    Serial.print(":");
    Serial.println(BROKER_PORT);
    Serial.print("Board ID: ");
    Serial.println(BOARD_ID);

    messenger.onMessage(onMessage);
    messenger.begin(
        WIFI_SSID,
        WIFI_PASSWORD,
        BROKER_HOST,
        BROKER_PORT,
        GROUP_ID,
        BOARD_ID
    );

    stopRobot();
}

void loop()
{
    messenger.loop();
    sendRegister();
    printConnectionStatus();

    if (
        safetyEnabled
        &&
        lastHeartbeatMs != 0
        &&
        (millis() - lastHeartbeatMs) > WIFI_HEARTBEAT_TIMEOUT_MS
    )
    {
        safetyEnabled = false;
        Serial.println("SAFETY: heartbeat timeout");
    }

    const bool enabled = movementEnabled();

    if (enabled)
    {
        if (stopped)
        {
            Serial.println("Safety enabled: driving forward");
        }

        stopped = false;
        driveForward();
    }
    else
    {
        if (!stopped)
        {
            Serial.print("Safety disabled: stopping");

            if (lastMessage[0] != '\0')
            {
                Serial.print(", message=");
                Serial.print(lastMessage);
            }

            Serial.println();
        }

        stopped = true;
        stopRobot();
    }

    if ((millis() - lastStatusPrintMs) >= 5000)
    {
        lastStatusPrintMs = millis();
        Serial.print("Status: ");
        Serial.print(enabled ? "FORWARD" : "STOPPED");
        Serial.print(", Safety ");
        Serial.print(safetyEnabled ? "ENABLED" : "DISABLED");
        Serial.print(", WiFi ");
        Serial.println(messenger.isConnected() ? "OK" : "NOT CONNECTED");
    }

    delay(50);
}
