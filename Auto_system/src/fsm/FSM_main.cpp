#if defined(CORE_CM7)

#include "FSM_main.h"

#include <Arduino.h>
#include <MiniMessenger.h>
#include <string.h>

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
#include "LidarSensor.h"
#include "RFIDHandler.h"
#include "ReviveButton.h"
#include "WiFiHandlerConfig.h"

namespace
{
constexpr uint32_t STATUS_PRINT_INTERVAL_MS = 5000;
constexpr uint32_t LOOP_DELAY_MS = 10;
constexpr uint32_t AIRLOCK_REQUEST_RETRY_MS = 1000;
constexpr uint32_t SOIL_QUERY_RETRY_MS = 1000;

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
LidarSensor lidar(LIDAR_SERIAL);
MiniMessenger messenger;

enum class PendingAirlockRequest : uint8_t
{
    None,
    ExitTunnelB,
    EntryTunnelA
};

bool safetyEnabled = false;
bool lastConnectedState = false;
bool missionStartRequested = false;
bool missionStarted = false;
bool driveReady = false;
bool soilQueryPending = false;
PendingAirlockRequest pendingAirlockRequest = PendingAirlockRequest::None;
uint32_t lastRegisterMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastStatusPrintMs = 0;
uint32_t lastAirlockRequestMs = 0;
uint32_t lastSoilQueryMs = 0;
uint8_t lastSeedsPlanted = 0;
char pendingSoilTagId[24] = "";
char activePlantTagId[24] = "";
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

bool robotIsInState(
    MissionState mission,
    BaseState base,
    ExitBaseState exitBase
)
{
    return
        fsm.missionState() == mission &&
        fsm.baseState() == base &&
        fsm.exitBaseState() == exitBase;
}

bool robotIsInState(
    MissionState mission,
    BaseState base,
    ReturnState returnState
)
{
    return
        fsm.missionState() == mission &&
        fsm.baseState() == base &&
        fsm.returnState() == returnState;
}

RobotApp::AutomaticMotionPhase automaticMotionPhase()
{
    if (robotIsInState(
            MissionState::Base,
            BaseState::ExitBase,
            ExitBaseState::LineFollowToDoor
        ))
    {
        return RobotApp::AutomaticMotionPhase::ExitLineToDoor;
    }

    if (robotIsInState(
            MissionState::Base,
            BaseState::ExitBase,
            ExitBaseState::WaitForDoor
        ))
    {
        return RobotApp::AutomaticMotionPhase::ExitWaitForDoor;
    }

    if (robotIsInState(
            MissionState::Base,
            BaseState::ExitBase,
            ExitBaseState::TraverseTunnel
        ))
    {
        return RobotApp::AutomaticMotionPhase::ExitTraverseTunnel;
    }

    if (robotIsInState(
            MissionState::Base,
            BaseState::ReturnHome,
            ReturnState::NavigateToAirlock
        ))
    {
        return RobotApp::AutomaticMotionPhase::ReturnToAirlock;
    }

    if (robotIsInState(
            MissionState::Base,
            BaseState::ReturnHome,
            ReturnState::WaitForEntryDoor
        ))
    {
        return RobotApp::AutomaticMotionPhase::EntryWaitForDoor;
    }

    if (robotIsInState(
            MissionState::Base,
            BaseState::ReturnHome,
            ReturnState::TraverseTunnel
        ))
    {
        return RobotApp::AutomaticMotionPhase::EntryTraverseTunnel;
    }

    return RobotApp::AutomaticMotionPhase::Idle;
}

void refreshAutomaticEventContext()
{
    RobotApp::updateAutomaticEventContext(
        automaticMotionPhase(),
        lidar.getDistanceCM(),
        lidar.isValid(),
        RobotNavigation::navigator().lineReading().visible
    );
}

bool sendServerMessage(const char* payload)
{
    return messenger.sendToBoard(SERVER_BOARD_ID, payload);
}

void sendAirlockRequest(
    PendingAirlockRequest request,
    const char* messageType
)
{
    if (pendingAirlockRequest != request)
    {
        pendingAirlockRequest = request;
        lastAirlockRequestMs = 0;
    }

    if (
        lastAirlockRequestMs != 0 &&
        millis() - lastAirlockRequestMs < AIRLOCK_REQUEST_RETRY_MS
    )
    {
        return;
    }

    char payload[96];
    snprintf(
        payload,
        sizeof(payload),
        "type=%s team_id=%s board_id=%s",
        messageType,
        GROUP_ID,
        BOARD_ID
    );

    if (sendServerMessage(payload))
    {
        lastAirlockRequestMs = millis();
    }
}

void serviceAirlockRequests()
{
    if (robotIsInState(
            MissionState::Base,
            BaseState::ExitBase,
            ExitBaseState::RequestClearance
        ))
    {
        sendAirlockRequest(
            PendingAirlockRequest::ExitTunnelB,
            "openAirlockB"
        );
        return;
    }

    if (robotIsInState(
            MissionState::Base,
            BaseState::ReturnHome,
            ReturnState::WaitForEntryDoor
        ))
    {
        sendAirlockRequest(
            PendingAirlockRequest::EntryTunnelA,
            "openAirlockA"
        );
        return;
    }

    if (pendingAirlockRequest != PendingAirlockRequest::None)
    {
        pendingAirlockRequest = PendingAirlockRequest::None;
        lastAirlockRequestMs = 0;
    }
}

void copyTagId(const String& source, char* destination, size_t destinationSize)
{
    source.toCharArray(destination, destinationSize);
}

void sendSoilQuery()
{
    if (pendingSoilTagId[0] == '\0')
    {
        return;
    }

    if (
        lastSoilQueryMs != 0 &&
        millis() - lastSoilQueryMs < SOIL_QUERY_RETRY_MS
    )
    {
        return;
    }

    char payload[128];
    snprintf(
        payload,
        sizeof(payload),
        "type=isFertile team_id=%s board_id=%s tag_id=%s",
        GROUP_ID,
        BOARD_ID,
        pendingSoilTagId
    );

    if (sendServerMessage(payload))
    {
        lastSoilQueryMs = millis();
    }
}

void rememberPlantTag(const char* tagId)
{
    if (tagId == nullptr)
    {
        activePlantTagId[0] = '\0';
        return;
    }

    strncpy(activePlantTagId, tagId, sizeof(activePlantTagId) - 1);
    activePlantTagId[sizeof(activePlantTagId) - 1] = '\0';
}

void sendSeedPlantedIfNeeded()
{
    const uint8_t seedsPlanted = fsm.seedsPlanted();

    if (seedsPlanted <= lastSeedsPlanted)
    {
        return;
    }

    lastSeedsPlanted = seedsPlanted;

    if (activePlantTagId[0] == '\0')
    {
        return;
    }

    char payload[128];
    snprintf(
        payload,
        sizeof(payload),
        "type=seedPlanted team_id=%s board_id=%s tag_id=%s",
        GROUP_ID,
        BOARD_ID,
        activePlantTagId
    );

    sendServerMessage(payload);
    activePlantTagId[0] = '\0';
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

    if (
        fsm.missionState() != MissionState::Grid ||
        fsm.gridState() != GridState::ExploreGrid ||
        fsm.exploreState() != ExploreState::DriveGrid ||
        soilQueryPending
    )
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
        if (fertile)
        {
            copyTagId(uid, activePlantTagId, sizeof(activePlantTagId));
        }
    }
    else
    {
        copyTagId(uid, pendingSoilTagId, sizeof(pendingSoilTagId));
        soilQueryPending = true;
        lastSoilQueryMs = 0;
        robot.stop_all();

        Serial.print("RFID server query: uid=");
        Serial.println(uid);
        sendSoilQuery();
    }
}

void handleRemoteEvents()
{
    if (remoteEvents.airlockReply)
    {
        if (remoteEvents.airlockAccepted)
        {
            if (pendingAirlockRequest == PendingAirlockRequest::ExitTunnelB)
            {
                RobotApp::notifyExitAirlockAccepted();
            }
            else if (
                pendingAirlockRequest == PendingAirlockRequest::EntryTunnelA
            )
            {
                RobotApp::notifyEntryAirlockAccepted();
            }
        }

        pendingAirlockRequest = PendingAirlockRequest::None;
        lastAirlockRequestMs = 0;
        remoteEvents.airlockReply = false;
        remoteEvents.airlockAccepted = false;
    }

    if (remoteEvents.soilReply)
    {
        char replyTagId[24];
        strncpy(replyTagId, pendingSoilTagId, sizeof(replyTagId) - 1);
        replyTagId[sizeof(replyTagId) - 1] = '\0';

        if (remoteEvents.rfidTagId[0] != '\0')
        {
            strncpy(replyTagId, remoteEvents.rfidTagId, sizeof(replyTagId) - 1);
            replyTagId[sizeof(replyTagId) - 1] = '\0';
        }

        soilQueryPending = false;
        pendingSoilTagId[0] = '\0';
        lastSoilQueryMs = 0;

        const bool shouldPlant =
            remoteEvents.rfidFertile && !remoteEvents.rfidAlreadyPlanted;

        if (remoteEvents.rfidCoordinate[0] != '\0')
        {
            RobotNavigation::navigator().observeRemoteRfidCoordinate(
                remoteEvents.rfidCoordinate,
                shouldPlant
            );
            fsm.rfidDetected(remoteEvents.rfidCoordinate, shouldPlant);

            if (shouldPlant)
            {
                if (replyTagId[0] != '\0')
                {
                    rememberPlantTag(replyTagId);
                }
            }
        }

        remoteEvents.soilReply = false;
        remoteEvents.rfidFertile = false;
        remoteEvents.rfidAlreadyPlanted = false;
        remoteEvents.rfidCoordinate[0] = '\0';
        remoteEvents.rfidTagId[0] = '\0';
    }

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
        missionStarted = false;
        remoteEvents.baseReached = false;
    }

    if (remoteEvents.stranded)
    {
        RobotApp::notifyRobotStranded();
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

    if (soilQueryPending)
    {
        robot.stop_all();
        sendSoilQuery();
        return;
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

    serviceAirlockRequests();
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
    lidar.begin();
    rfidReader.begin();
    RobotNavigation::navigator().begin();
    fsm.setNavigator(&RobotNavigation::navigator());
    fsm.begin();
    lastSeedsPlanted = fsm.seedsPlanted();

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
    Serial.println("MQTT uplink: register, openAirlockA/B, isFertile, seedPlanted");
    Serial.println("MQTT downlink: heartbeat, openAirlockReply, isFertileReply, emergency, disable, revive");
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
    if (lidar.update())
    {
        RobotNavigation::navigator().observeObstacleDistance(
            lidar.getDistanceCM()
        );
    }
    RobotNavigation::navigator().observeLine(irSensors);
    refreshAutomaticEventContext();

    handleRemoteEvents();

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

    handleAutomaticEvents();
    if (soilQueryPending)
    {
        updateStatusLed();
        printStatus();
        delay(LOOP_DELAY_MS);
        return;
    }

    fsm.update();
    sendSeedPlantedIfNeeded();
    updateStatusLed();
    printStatus();

    delay(LOOP_DELAY_MS);
}
} // namespace RobotApp

#endif
