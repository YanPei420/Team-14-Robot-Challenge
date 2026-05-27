#if defined(CORE_CM7)

#include "FSM_main.h"

#include <Arduino.h>
#include <MiniMessenger.h>

#include "../M7/M7DriveProxy.h"
#include "../RobotFSM.h"
#include "../navigation/NavigationRuntime.h"
#include "FSM_auto_events.h"
#include "FSM_remote_events.h"
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
RobotApp::RemoteMessageParser remoteParser;
RobotApp::RemoteEvents remoteEvents;

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

void onMessage(
    const MessageMetadata& metadata,
    const uint8_t* payload,
    size_t length
)
{
    (void) metadata;

    remoteParser.copyPayload(payload, length);

    if (remoteParser.messageFieldEquals("type", "heartbeat"))
    {
        lastHeartbeatMs = millis();

        if (remoteParser.messageFieldEquals("enable", "1"))
        {
            safetyEnabled = true;
            missionStartRequested = AUTO_START_WHEN_SAFETY_ENABLED;
        }
        else if (remoteParser.messageFieldEquals("enable", "0"))
        {
            safetyEnabled = false;
            Serial.println("SAFETY: heartbeat disabled");
        }
    }

    if (
        remoteParser.messageFieldEquals("type", "stop") ||
        remoteParser.messageFieldEquals("type", "emergency") ||
        remoteParser.messageFieldEquals("type", "disable")
    )
    {
        safetyEnabled = false;
        Serial.print("SAFETY: stop command received, message=");
        Serial.println(remoteParser.message());
    }

    remoteParser.parseInto(remoteEvents);
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

void handleRfidEvent()
{
    if (!rfidReader.update())
    {
        return;
    }

    char coordinate[4];
    bool fertile = false;
    const String uid = rfidReader.getUID();

    if (RobotApp::lookupRfidTag(uid, coordinate, sizeof(coordinate), fertile))
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

        RobotNavigation::navigator().observeRemoteRfidCoordinate(
            coordinate,
            remoteEvents.rfidFertile
        );
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

    if (RobotApp::exitClearanceReceivedAutomatically())
    {
        fsm.exitClearanceReceived();
    }

    if (RobotApp::exitDoorDetectedAutomatically())
    {
        fsm.exitDoorDetected();
    }

    if (RobotApp::exitDoorOpenedAutomatically())
    {
        fsm.exitDoorOpened();
    }

    if (RobotApp::mainArenaReachedAutomatically())
    {
        fsm.mainArenaReached();
    }

    handleRfidEvent();

    if (RobotApp::entryAirlockReachedAutomatically())
    {
        fsm.entryAirlockReached();
    }

    if (RobotApp::entryDoorOpenedAutomatically())
    {
        fsm.entryDoorOpened();
    }

    if (RobotApp::baseReachedAutomatically())
    {
        fsm.baseReached();
        missionStarted = false;
    }

    if (RobotApp::robotIsStrandedAutomatically())
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
    RobotNavigation::navigator().begin();
    fsm.setNavigator(&RobotNavigation::navigator());
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
    RobotNavigation::navigator().observeLine(irSensors);

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
