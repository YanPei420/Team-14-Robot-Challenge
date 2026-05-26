#if defined(CORE_CM7)

#include "FSM_main.h"

#include <Arduino.h>
#include <MiniMessenger.h>
#include <string.h>

#include "../M7/M7DriveProxy.h"
#include "../RobotFSM.h"
#include "IRConfig.h"
#include "IRSensor.h"
#include "KillSwitch.h"
#include "KillSwitchConfig.h"
#include "LED.h"
#include "RFIDHandler.h"
#include "ReviveButton.h"
#include "WiFiHandlerConfig.h"

namespace
{
constexpr uint32_t STATUS_PRINT_INTERVAL_MS = 5000;
constexpr uint32_t LOOP_DELAY_MS = 10;

// Keep this true for the challenge run: once the server heartbeat enables
// movement, the FSM leaves Base/Idle automatically.
constexpr bool AUTO_START_WHEN_SAFETY_ENABLED = true;

M7DriveProxy robot;
RobotFSM fsm(robot);
KillSwitch killSwitch(KILL_SWITCH_PIN);
LED statusLed;
ReviveButton reviveButton;
RFIDHandler rfidReader;
IRSensor irSensors(IR_PINS, IR_SENSOR_COUNT);
MiniMessenger messenger;

bool safetyEnabled = false;
bool lastConnectedState = false;
bool missionStartRequested = false;
bool missionStarted = false;
bool driveReady = false;
uint32_t lastRegisterMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastStatusPrintMs = 0;
char lastMessage[MiniMessenger::kMaxPayloadSize + 1] = "";

struct RemoteEvents
{
    bool start = false;
    bool exitClearance = false;
    bool exitDoorDetected = false;
    bool exitDoorOpened = false;
    bool mainArenaReached = false;
    bool emergencyReturn = false;
    bool entryAirlockReached = false;
    bool entryDoorOpened = false;
    bool baseReached = false;
    bool stranded = false;
    bool revive = false;
    bool rfid = false;
    bool rfidFertile = false;
    char rfidCoordinate[4] = "";
};

RemoteEvents remoteEvents;

bool messageContains(const char* token)
{
    return strstr(lastMessage, token) != nullptr;
}

bool messageFieldEquals(const char* key, const char* value)
{
    char token[48];
    snprintf(token, sizeof(token), "%s=%s", key, value);
    return messageContains(token);
}

void copyCoordinateFromMessage(char* destination, size_t destinationSize)
{
    destination[0] = '\0';

    const char* coordinateStart = strstr(lastMessage, "coordinate=");
    if (coordinateStart == nullptr)
    {
        return;
    }

    coordinateStart += strlen("coordinate=");
    size_t i = 0;

    while (
        coordinateStart[i] != '\0' &&
        coordinateStart[i] != ' ' &&
        i + 1 < destinationSize
    )
    {
        destination[i] = coordinateStart[i];
        ++i;
    }

    destination[i] = '\0';
}

bool heartbeatIsFresh()
{
    return
        lastHeartbeatMs != 0 &&
        (millis() - lastHeartbeatMs) <= WIFI_HEARTBEAT_TIMEOUT_MS;
}

bool remoteSafetyAllowsMotion()
{
    return driveReady && safetyEnabled && heartbeatIsFresh();
}

void parseRemoteEvent()
{
    if (messageFieldEquals("type", "start"))
    {
        remoteEvents.start = true;
    }
    else if (messageFieldEquals("type", "clearance"))
    {
        remoteEvents.exitClearance = true;
    }
    else if (messageFieldEquals("type", "exit_door_detected"))
    {
        remoteEvents.exitDoorDetected = true;
    }
    else if (messageFieldEquals("type", "exit_door_opened"))
    {
        remoteEvents.exitDoorOpened = true;
    }
    else if (messageFieldEquals("type", "arena_reached"))
    {
        remoteEvents.mainArenaReached = true;
    }
    else if (messageFieldEquals("type", "return"))
    {
        remoteEvents.emergencyReturn = true;
    }
    else if (messageFieldEquals("type", "entry_airlock_reached"))
    {
        remoteEvents.entryAirlockReached = true;
    }
    else if (messageFieldEquals("type", "entry_door_opened"))
    {
        remoteEvents.entryDoorOpened = true;
    }
    else if (messageFieldEquals("type", "base_reached"))
    {
        remoteEvents.baseReached = true;
    }
    else if (messageFieldEquals("type", "stranded"))
    {
        remoteEvents.stranded = true;
    }
    else if (messageFieldEquals("type", "revive"))
    {
        remoteEvents.revive = true;
    }
    else if (messageFieldEquals("type", "rfid"))
    {
        remoteEvents.rfid = true;
        remoteEvents.rfidFertile =
            messageFieldEquals("fertile", "1") ||
            messageFieldEquals("fertile", "true");
        copyCoordinateFromMessage(
            remoteEvents.rfidCoordinate,
            sizeof(remoteEvents.rfidCoordinate)
        );
    }
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

    if (messageFieldEquals("type", "heartbeat"))
    {
        lastHeartbeatMs = millis();

        if (messageFieldEquals("enable", "1"))
        {
            safetyEnabled = true;
            missionStartRequested = AUTO_START_WHEN_SAFETY_ENABLED;
        }
        else if (messageFieldEquals("enable", "0"))
        {
            safetyEnabled = false;
            Serial.println("SAFETY: heartbeat disabled");
        }
    }

    if (
        messageFieldEquals("type", "stop") ||
        messageFieldEquals("type", "emergency") ||
        messageFieldEquals("type", "disable")
    )
    {
        safetyEnabled = false;
        Serial.print("SAFETY: stop command received, message=");
        Serial.println(lastMessage);
    }

    parseRemoteEvent();
}

void sendRegister()
{
    if (
        lastRegisterMs != 0 &&
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

void updateHeartbeatSafety()
{
    if (
        safetyEnabled &&
        lastHeartbeatMs != 0 &&
        (millis() - lastHeartbeatMs) > WIFI_HEARTBEAT_TIMEOUT_MS
    )
    {
        safetyEnabled = false;
        Serial.println("SAFETY: heartbeat timeout");
    }
}

bool exitClearanceReceivedAutomatically()
{
    // TODO: Replace this with the official base/server clearance signal.
    return false;
}

bool exitDoorDetectedAutomatically()
{
    // TODO: Calibrate IR/LiDAR threshold for "robot is at the exit door".
    return false;
}

bool exitDoorOpenedAutomatically()
{
    // TODO: Add door-open detection from distance sensor, switch, or server event.
    return false;
}

bool mainArenaReachedAutomatically()
{
    // TODO: Detect arena entry using line markers, encoder distance, RFID, or beacon.
    return false;
}

bool entryAirlockReachedAutomatically()
{
    // TODO: Detect return airlock position using line markers, LiDAR, or beacon.
    return false;
}

bool entryDoorOpenedAutomatically()
{
    // TODO: Add return-door-open detection from distance sensor, switch, or server event.
    return false;
}

bool baseReachedAutomatically()
{
    // TODO: Detect inside-base condition using marker, encoder distance, or server event.
    return false;
}

bool robotIsStrandedAutomatically()
{
    // TODO: Detect stranded condition from no encoder progress, timeout, or server rescue event.
    return false;
}

bool lookupRfidTag(
    const String& uid,
    char* coordinate,
    size_t coordinateSize,
    bool& fertile
)
{
    (void) uid;

    // TODO: Replace this placeholder with the real UID-to-coordinate/soil map.
    // Example:
    // if (uid == "12 34 56 78")
    // {
    //     strncpy(coordinate, "A1", coordinateSize);
    //     coordinate[coordinateSize - 1] = '\0';
    //     fertile = true;
    //     return true;
    // }

    coordinate[0] = '\0';
    fertile = false;
    return false;
}

void handleRfidEvent()
{
    if (!rfidReader.update())
    {
        return;
    }

    char coordinate[4];
    bool fertile = false;
    const String uid = rfidReader.getUID();

    if (lookupRfidTag(uid, coordinate, sizeof(coordinate), fertile))
    {
        Serial.print("RFID mapped: uid=");
        Serial.print(uid);
        Serial.print(" coordinate=");
        Serial.print(coordinate);
        Serial.print(" fertile=");
        Serial.println(fertile ? "true" : "false");

        fsm.rfidDetected(coordinate, fertile);
    }
    else
    {
        Serial.print("TODO: unmapped RFID uid=");
        Serial.println(uid);
    }
}

void handleRemoteEvents()
{
    if (remoteEvents.start)
    {
        missionStartRequested = true;
        remoteEvents.start = false;
    }

    if (remoteEvents.exitClearance)
    {
        fsm.exitClearanceReceived();
        remoteEvents.exitClearance = false;
    }

    if (remoteEvents.exitDoorDetected)
    {
        fsm.exitDoorDetected();
        remoteEvents.exitDoorDetected = false;
    }

    if (remoteEvents.exitDoorOpened)
    {
        fsm.exitDoorOpened();
        remoteEvents.exitDoorOpened = false;
    }

    if (remoteEvents.mainArenaReached)
    {
        fsm.mainArenaReached();
        remoteEvents.mainArenaReached = false;
    }

    if (remoteEvents.rfid)
    {
        const char* coordinate =
            remoteEvents.rfidCoordinate[0] != '\0'
            ?
            remoteEvents.rfidCoordinate
            :
            "??";

        fsm.rfidDetected(coordinate, remoteEvents.rfidFertile);
        remoteEvents.rfid = false;
    }

    if (remoteEvents.emergencyReturn)
    {
        fsm.emergencyWarningReceived();
        remoteEvents.emergencyReturn = false;
    }

    if (remoteEvents.entryAirlockReached)
    {
        fsm.entryAirlockReached();
        remoteEvents.entryAirlockReached = false;
    }

    if (remoteEvents.entryDoorOpened)
    {
        fsm.entryDoorOpened();
        remoteEvents.entryDoorOpened = false;
    }

    if (remoteEvents.baseReached)
    {
        fsm.baseReached();
        remoteEvents.baseReached = false;
    }

    if (remoteEvents.stranded)
    {
        fsm.markStranded();
        remoteEvents.stranded = false;
    }

    if (remoteEvents.revive)
    {
        fsm.reviveFromStranded();
        remoteEvents.revive = false;
    }
}

void handleAutomaticEvents()
{
    if (missionStartRequested && !missionStarted)
    {
        fsm.startMission();
        missionStarted = true;
        missionStartRequested = false;
    }

    if (exitClearanceReceivedAutomatically())
    {
        fsm.exitClearanceReceived();
    }

    if (exitDoorDetectedAutomatically())
    {
        fsm.exitDoorDetected();
    }

    if (exitDoorOpenedAutomatically())
    {
        fsm.exitDoorOpened();
    }

    if (mainArenaReachedAutomatically())
    {
        fsm.mainArenaReached();
    }

    handleRfidEvent();

    if (entryAirlockReachedAutomatically())
    {
        fsm.entryAirlockReached();
    }

    if (entryDoorOpenedAutomatically())
    {
        fsm.entryDoorOpened();
    }

    if (baseReachedAutomatically())
    {
        fsm.baseReached();
        missionStarted = false;
    }

    if (robotIsStrandedAutomatically())
    {
        fsm.markStranded();
    }
}

void updateStatusLed()
{
    if (killSwitch.isTriggered() || fsm.isEmergencyStop() ||
        !remoteSafetyAllowsMotion())
    {
        statusLed.showEmergency();
    }
    else if (reviveButton.isPressed())
    {
        statusLed.showButtonPressed();
    }
    else
    {
        statusLed.showNormal();
    }
}

void printStatus()
{
    if ((millis() - lastStatusPrintMs) < STATUS_PRINT_INTERVAL_MS)
    {
        return;
    }

    lastStatusPrintMs = millis();

    Serial.print("Status: ");
    Serial.print(fsm.stateName());
    Serial.print(" | safety=");
    Serial.print(remoteSafetyAllowsMotion() ? "enabled" : "disabled");
    Serial.print(" | M4 drive=");
    Serial.print(driveReady ? "ok" : "offline");
    Serial.print(" | WiFi=");
    Serial.print(messenger.isConnected() ? "ok" : "offline");
    Serial.print(" | seeds=");
    Serial.print(fsm.seedsPlanted());
    Serial.print('/');
    Serial.println(fsm.seedsPlanted() + fsm.seedsRemaining());
}
} // namespace

namespace RobotApp
{
void fsmSetup()
{
    driveReady = robot.begin();
    if (!driveReady)
    {
        Serial.println("M4 chassis service not ready; motion disabled");
    }
    robot.stop_all();

    killSwitch.begin();
    statusLed.begin();
    reviveButton.begin();
    irSensors.begin();
    rfidReader.begin();
    fsm.begin();

    messenger.onMessage(onMessage);
    messenger.begin(
        WIFI_SSID,
        WIFI_PASSWORD,
        BROKER_HOST,
        BROKER_PORT,
        GROUP_ID,
        BOARD_ID
    );

    Serial.println("TERM 3 CHALLENGE AUTOMATIC FSM READY");
    Serial.println("M7: WiFi/MQTT, safety gate, RFID events, FSM");
    Serial.println("M4: Motoron chassis, motor refresh, wheel encoders");
    Serial.println("MQTT events: start, clearance, exit_door_detected, exit_door_opened, arena_reached, rfid, return, entry_airlock_reached, entry_door_opened, base_reached, stranded, revive");
    Serial.println("TODO: calibrate automatic door, arena, base, stranded, and RFID map detection functions in src/fsm/FSM_main.cpp.");
}

void fsmLoop()
{
    messenger.loop();
    sendRegister();
    printConnectionStatus();
    updateHeartbeatSafety();

    killSwitch.update();
    reviveButton.update();
    irSensors.update();

    if (killSwitch.isTriggered() || !remoteSafetyAllowsMotion())
    {
        fsm.triggerEmergencyStop();
        updateStatusLed();
        printStatus();
        delay(LOOP_DELAY_MS);
        return;
    }

    fsm.clearEmergencyStop();

    if (reviveButton.isPressed())
    {
        fsm.reviveFromStranded();
    }

    handleRemoteEvents();
    handleAutomaticEvents();

    fsm.update();
    updateStatusLed();
    printStatus();

    delay(LOOP_DELAY_MS);
}
} // namespace RobotApp

#endif
