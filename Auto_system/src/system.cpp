#include "system.h"

#include <Arduino.h>

#if defined(CORE_CM7)

#include <Servo.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "IRConfig.h"
#include "IRSensor.h"
#include "KillSwitch.h"
#include "LED.h"
#include "LineFollower.h"
#include "LidarSensor.h"
#include "MiniMessenger.h"
#include "MotorConfig.h"
#include "MotoronDrive.h"
#include "RFIDHandler.h"
#include "ReviveButton.h"
#include "ServoConfig.h"
#include "WallFollower.h"
#include "WiFiHandlerConfig.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t STATUS_INTERVAL_MS = 500;
constexpr uint32_t LIDAR_FRESH_MS = 350;

constexpr int16_t LINE_SPEED = 220;
constexpr int16_t GRID_SPEED = 210;
constexpr int16_t RETURN_SPEED = 220;
constexpr int16_t TUNNEL_SPEED = 230;
constexpr int16_t FINE_ADJUST_SPEED = 120;
constexpr int16_t AVOID_TURN_SPEED = 260;
constexpr int16_t MANUAL_SPEED_DEFAULT = 180;
constexpr int16_t MANUAL_SPEED_STEP = 40;
constexpr int16_t MANUAL_SPEED_MIN = 80;
constexpr int16_t MANUAL_SPEED_MAX = 500;
constexpr uint32_t MANUAL_COMMAND_TIMEOUT_MS = 1500;

constexpr uint16_t DOOR_DETECTED_DISTANCE_CM = 28;
constexpr uint16_t DOOR_OPEN_DISTANCE_CM = 70;
constexpr uint16_t OBSTACLE_DISTANCE_CM = 22;
constexpr uint32_t DOOR_STABLE_MS = 250;
constexpr uint32_t DOOR_OPEN_STABLE_MS = 250;

constexpr uint32_t TUNNEL_TRAVERSE_MS = 3500;
constexpr uint32_t ENTRY_TUNNEL_TRAVERSE_MS = 3500;
constexpr uint32_t ENTRY_ACCEPT_FALLBACK_MS = 1500;
constexpr uint32_t ARENA_TIME_LIMIT_MS = 4UL * 60UL * 1000UL;

constexpr uint32_t SERVER_REQUEST_RETRY_MS = 1000;
constexpr uint32_t SOIL_QUERY_TIMEOUT_MS = 2500;
constexpr uint32_t ALIGN_SEARCH_MS = 500;
constexpr uint32_t FINE_ADJUST_MS = 700;
constexpr uint32_t HOPPER_OPEN_MS = 500;
constexpr uint32_t DROP_SEED_MS = 700;
constexpr uint32_t PLANT_VERIFY_MS = 300;
constexpr uint32_t RFID_COOLDOWN_MS = 2000;

constexpr uint8_t MAX_SEEDS = 5;
constexpr int HOPPER_CLOSED_ANGLE = SERVO_SWEEP_MIN;
constexpr int HOPPER_OPEN_ANGLE = SERVO_SWEEP_MAX;

enum class RunState : uint8_t
{
    Idle,
    ExitLineToDoor,
    ExitRequest,
    ExitWaitDoor,
    ExitTraverseTunnel,
    GridDrive,
    SoilQuery,
    AlignSearch,
    FineAdjust,
    PlantOpen,
    PlantDrop,
    PlantVerify,
    ReturnToAirlock,
    EntryRequest,
    EntryWaitDoor,
    EntryTraverseTunnel,
    InsideBase,
    Stranded,
    ManualControl,
    Finished
};

enum class ManualCommand : uint8_t
{
    Stop,
    Forward,
    Backward,
    Left,
    Right,
    RotateLeft,
    RotateRight,
    LineFollow,
    WallFollow
};

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
IRSensor lineSensors(IR_PINS, IR_SENSOR_COUNT);
LidarSensor lidarLeft(LIDAR_SERIAL_1);
LidarSensor lidarRight(LIDAR_SERIAL_2);
LidarSensor lidarFront(LIDAR_SERIAL_3);
LineFollower lineFollower(robot, lineSensors);
WallFollower tunnelFollower(robot, lidarLeft, lidarRight);
RFIDHandler rfid;
KillSwitch killSwitch;
ReviveButton reviveButton;
LED statusLed;
MiniMessenger messenger;
Servo hopperServo;

RunState state = RunState::Idle;

bool motorReady = false;
bool encoderControlReady = false;
bool remoteSafetyEnabled = false;
bool startRequested = false;
bool remoteReturnRequested = false;
bool remoteStrandedRequested = false;
bool remoteReviveRequested = false;
bool baseReachedRequested = false;
bool exitAirlockAccepted = false;
bool entryAirlockAccepted = false;
bool soilResponseReceived = false;
bool soilFertile = false;
bool soilAlreadyPlanted = false;
bool missionCompleteSent = false;
bool rfidDetectedThisLoop = false;

uint8_t seedsPlanted = 0;

ManualCommand manualCommand = ManualCommand::Stop;
int16_t manualSpeed = MANUAL_SPEED_DEFAULT;

uint32_t lastHeartbeatMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastRegisterMs = 0;
uint32_t stateStartedMs = 0;
uint32_t arenaStartedMs = 0;
uint32_t doorNearSinceMs = 0;
uint32_t doorOpenSinceMs = 0;
uint32_t lastServerRequestMs = 0;
uint32_t lastRfidMs = 0;
uint32_t lastManualCommandMs = 0;

char detectedRfidUid[40] = {0};
char pendingTagId[40] = {0};
char lastRfidUid[40] = {0};

bool safetyAllowed();
bool heartbeatOk();

const char* stateName(RunState value)
{
    switch (value)
    {
        case RunState::Idle:
            return "Idle";
        case RunState::ExitLineToDoor:
            return "ExitLineToDoor";
        case RunState::ExitRequest:
            return "ExitRequest";
        case RunState::ExitWaitDoor:
            return "ExitWaitDoor";
        case RunState::ExitTraverseTunnel:
            return "ExitTraverseTunnel";
        case RunState::GridDrive:
            return "GridDrive";
        case RunState::SoilQuery:
            return "SoilQuery";
        case RunState::AlignSearch:
            return "AlignSearch";
        case RunState::FineAdjust:
            return "FineAdjust";
        case RunState::PlantOpen:
            return "PlantOpen";
        case RunState::PlantDrop:
            return "PlantDrop";
        case RunState::PlantVerify:
            return "PlantVerify";
        case RunState::ReturnToAirlock:
            return "ReturnToAirlock";
        case RunState::EntryRequest:
            return "EntryRequest";
        case RunState::EntryWaitDoor:
            return "EntryWaitDoor";
        case RunState::EntryTraverseTunnel:
            return "EntryTraverseTunnel";
        case RunState::InsideBase:
            return "InsideBase";
        case RunState::Stranded:
            return "Stranded";
        case RunState::ManualControl:
            return "ManualControl";
        case RunState::Finished:
            return "Finished";
    }

    return "Unknown";
}

void stopRobot()
{
    robot.stop_all();
}

void printSerialHelp()
{
    Serial.println("Serial commands:");
    Serial.println("  S - start autonomous mission");
    Serial.println("  R - request return to base");
    Serial.println("  X - disable remote safety and stop");
    Serial.println("  M - enter manual control");
    Serial.println("  P - leave manual control and stop in Idle");
    Serial.println("  W/B/A/D - manual forward/back/left/right");
    Serial.println("  Q/E - manual rotate left/right");
    Serial.println("  L - manual line follow");
    Serial.println("  G - manual wall follow");
    Serial.println("  0 or Space - manual stop");
    Serial.println("  + / - - manual speed up/down");
    Serial.println("  ? - print this help");
}

void closeHopper()
{
    hopperServo.write(HOPPER_CLOSED_ANGLE);
}

void openHopper()
{
    hopperServo.write(HOPPER_OPEN_ANGLE);
}

void resetLineControl()
{
    lineFollower.reset();
    tunnelFollower.reset();
}

void resetDoorStableTimers()
{
    doorNearSinceMs = 0;
    doorOpenSinceMs = 0;
}

bool stateElapsed(uint32_t durationMs)
{
    return millis() - stateStartedMs >= durationMs;
}

bool lidarFresh(LidarSensor& lidar)
{
    return lidar.isValid() && millis() - lidar.getLastUpdateMs() <= LIDAR_FRESH_MS;
}

bool frontDistanceInRange(uint16_t distanceCM)
{
    return lidarFresh(lidarFront)
        && lidarFront.getDistanceCM() > 0
        && lidarFront.getDistanceCM() <= static_cast<int16_t>(distanceCM);
}

bool frontDistanceClear(uint16_t distanceCM)
{
    return lidarFresh(lidarFront)
        && lidarFront.getDistanceCM() >= static_cast<int16_t>(distanceCM);
}

bool stableFor(bool condition, uint32_t& sinceMs, uint32_t stableMs)
{
    const uint32_t now = millis();

    if (!condition)
    {
        sinceMs = 0;
        return false;
    }

    if (sinceMs == 0)
    {
        sinceMs = now;
        return false;
    }

    return now - sinceMs >= stableMs;
}

bool doorNearStable()
{
    return stableFor(
        frontDistanceInRange(DOOR_DETECTED_DISTANCE_CM),
        doorNearSinceMs,
        DOOR_STABLE_MS
    );
}

bool doorOpenStable()
{
    return stableFor(
        frontDistanceClear(DOOR_OPEN_DISTANCE_CM),
        doorOpenSinceMs,
        DOOR_OPEN_STABLE_MS
    );
}

bool frontObstacleBlocked()
{
    return frontDistanceInRange(OBSTACLE_DISTANCE_CM);
}

bool extractValue(
    const char* text,
    const char* key,
    char* output,
    size_t outputSize
)
{
    if (!text || !key || !output || outputSize == 0)
    {
        return false;
    }

    const char* start = strstr(text, key);
    if (!start)
    {
        output[0] = '\0';
        return false;
    }

    start += strlen(key);

    size_t index = 0;
    while (
        start[index] != '\0' &&
        start[index] != ' ' &&
        start[index] != '\r' &&
        start[index] != '\n' &&
        index + 1 < outputSize
    )
    {
        output[index] = start[index];
        ++index;
    }

    output[index] = '\0';
    return index > 0;
}

bool messageTypeIs(const char* text, const char* type)
{
    char messageType[32] = {0};
    return extractValue(text, "type=", messageType, sizeof(messageType))
        && strcmp(messageType, type) == 0;
}

bool tokenTrue(const char* text, const char* key)
{
    char value[12] = {0};
    if (!extractValue(text, key, value, sizeof(value)))
    {
        return false;
    }

    return strcmp(value, "1") == 0 || strcmp(value, "true") == 0;
}

bool tokenFalse(const char* text, const char* key)
{
    char value[12] = {0};
    if (!extractValue(text, key, value, sizeof(value)))
    {
        return false;
    }

    return strcmp(value, "0") == 0 || strcmp(value, "false") == 0;
}

bool sendToServer(const char* payload)
{
    if (!payload)
    {
        return false;
    }

    const bool sent = messenger.sendToBoard(SERVER_BOARD_ID, payload);

    Serial.print("[mqtt] ");
    Serial.print(sent ? "sent " : "queued/failed ");
    Serial.println(payload);

    return sent;
}

void sendRegister(bool force = false)
{
    const uint32_t now = millis();

    if (!force && now - lastRegisterMs < WIFI_REGISTER_INTERVAL_MS)
    {
        return;
    }

    char message[96];
    snprintf(
        message,
        sizeof(message),
        "type=register team_id=%s board_id=%s",
        GROUP_ID,
        BOARD_ID
    );

    sendToServer(message);
    lastRegisterMs = now;
}

void sendStatus(bool force = false)
{
    const uint32_t now = millis();

    if (!force && now - lastStatusMs < STATUS_INTERVAL_MS)
    {
        return;
    }

    char message[180];
    snprintf(
        message,
        sizeof(message),
        "type=status team_id=%s board_id=%s state=%s seeds=%u safety=%u",
        GROUP_ID,
        BOARD_ID,
        stateName(state),
        seedsPlanted,
        safetyAllowed() ? 1 : 0
    );

    sendToServer(message);

    Serial.print("[status] ");
    Serial.print(stateName(state));
    Serial.print(" seeds=");
    Serial.print(seedsPlanted);
    Serial.print(" motor=");
    Serial.print(motorReady ? "ok" : "fail");
    Serial.print(" encoder=");
    Serial.print(encoderControlReady ? "ok" : "fail");
    Serial.print(" kill=");
    Serial.print(killSwitch.isSafe() ? "safe" : "triggered");
    Serial.print(" heartbeat=");
    Serial.print(heartbeatOk() ? "ok" : "missing");
    Serial.print(" lidarF=");
    if (lidarFresh(lidarFront))
    {
        Serial.print(lidarFront.getDistanceCM());
        Serial.println("cm");
    }
    else
    {
        Serial.println("stale");
    }

    lastStatusMs = now;
}

void sendOpenAirlockB()
{
    char message[96];
    snprintf(
        message,
        sizeof(message),
        "type=openAirlockB team_id=%s board_id=%s",
        GROUP_ID,
        BOARD_ID
    );
    sendToServer(message);
    lastServerRequestMs = millis();
}

void sendOpenAirlockA()
{
    char message[96];
    snprintf(
        message,
        sizeof(message),
        "type=openAirlockA team_id=%s board_id=%s",
        GROUP_ID,
        BOARD_ID
    );
    sendToServer(message);
    lastServerRequestMs = millis();
}

void sendIsFertile()
{
    if (pendingTagId[0] == '\0')
    {
        return;
    }

    char message[140];
    snprintf(
        message,
        sizeof(message),
        "type=isFertile team_id=%s board_id=%s tag_id=%s",
        GROUP_ID,
        BOARD_ID,
        pendingTagId
    );
    sendToServer(message);
    lastServerRequestMs = millis();
}

void sendSeedPlanted()
{
    char message[160];
    snprintf(
        message,
        sizeof(message),
        "type=seedPlanted team_id=%s board_id=%s tag_id=%s count=%u",
        GROUP_ID,
        BOARD_ID,
        pendingTagId[0] == '\0' ? "unknown" : pendingTagId,
        seedsPlanted
    );
    sendToServer(message);
}

void sendMissionComplete()
{
    char message[120];
    snprintf(
        message,
        sizeof(message),
        "type=missionComplete team_id=%s board_id=%s seeds=%u",
        GROUP_ID,
        BOARD_ID,
        seedsPlanted
    );
    sendToServer(message);
}

void resetMission()
{
    seedsPlanted = 0;
    arenaStartedMs = 0;
    missionCompleteSent = false;
    remoteReturnRequested = false;
    remoteStrandedRequested = false;
    remoteReviveRequested = false;
    baseReachedRequested = false;
    exitAirlockAccepted = false;
    entryAirlockAccepted = false;
    soilResponseReceived = false;
    soilFertile = false;
    soilAlreadyPlanted = false;
    pendingTagId[0] = '\0';
    lastRfidUid[0] = '\0';
    closeHopper();
}

void setState(RunState next)
{
    if (state == next)
    {
        return;
    }

    state = next;
    stateStartedMs = millis();
    resetLineControl();
    resetDoorStableTimers();

    Serial.print("[state] ");
    Serial.println(stateName(state));

    switch (state)
    {
        case RunState::Idle:
        case RunState::ExitRequest:
        case RunState::ExitWaitDoor:
        case RunState::SoilQuery:
        case RunState::AlignSearch:
        case RunState::PlantOpen:
        case RunState::PlantDrop:
        case RunState::PlantVerify:
        case RunState::EntryRequest:
        case RunState::EntryWaitDoor:
        case RunState::InsideBase:
        case RunState::Stranded:
        case RunState::ManualControl:
        case RunState::Finished:
            stopRobot();
            break;

        case RunState::ExitLineToDoor:
        case RunState::ExitTraverseTunnel:
        case RunState::GridDrive:
        case RunState::FineAdjust:
        case RunState::ReturnToAirlock:
        case RunState::EntryTraverseTunnel:
            break;
    }

    if (state == RunState::ExitRequest)
    {
        exitAirlockAccepted = false;
        sendOpenAirlockB();
    }
    else if (state == RunState::SoilQuery)
    {
        soilResponseReceived = false;
        soilFertile = false;
        soilAlreadyPlanted = false;
        sendIsFertile();
    }
    else if (state == RunState::PlantOpen)
    {
        openHopper();
    }
    else if (state == RunState::PlantVerify)
    {
        closeHopper();
    }
    else if (state == RunState::EntryRequest)
    {
        entryAirlockAccepted = false;
        sendOpenAirlockA();
    }
    else if (state == RunState::InsideBase)
    {
        stopRobot();
        closeHopper();

        if (!missionCompleteSent)
        {
            sendMissionComplete();
            missionCompleteSent = true;
        }
    }
    else if (state == RunState::ManualControl)
    {
        manualCommand = ManualCommand::Stop;
        lastManualCommandMs = 0;
    }
}

bool heartbeatOk()
{
    return remoteSafetyEnabled
        && lastHeartbeatMs != 0
        && millis() - lastHeartbeatMs <= WIFI_HEARTBEAT_TIMEOUT_MS;
}

bool safetyAllowed()
{
    return motorReady
        && encoderControlReady
        && killSwitch.isSafe()
        && heartbeatOk();
}

bool isArenaOrReturnState()
{
    switch (state)
    {
        case RunState::GridDrive:
        case RunState::SoilQuery:
        case RunState::AlignSearch:
        case RunState::FineAdjust:
        case RunState::PlantOpen:
        case RunState::PlantDrop:
        case RunState::PlantVerify:
        case RunState::ReturnToAirlock:
        case RunState::EntryRequest:
        case RunState::EntryWaitDoor:
        case RunState::EntryTraverseTunnel:
            return true;

        default:
            return false;
    }
}

bool isManualMovementCommand(char command)
{
    switch (command)
    {
        case 'w':
        case 'W':
        case 'b':
        case 'B':
        case 'a':
        case 'A':
        case 'd':
        case 'D':
        case 'q':
        case 'Q':
        case 'e':
        case 'E':
        case 'l':
        case 'L':
        case 'g':
        case 'G':
        case '0':
        case ' ':
            return true;

        default:
            return false;
    }
}

void enterManualControl()
{
    startRequested = false;
    remoteReturnRequested = false;
    manualCommand = ManualCommand::Stop;
    lastManualCommandMs = 0;
    setState(RunState::ManualControl);
}

void setManualCommand(ManualCommand command)
{
    if (state != RunState::ManualControl)
    {
        enterManualControl();
    }

    if (command != manualCommand)
    {
        if (command == ManualCommand::LineFollow)
        {
            lineFollower.reset();
        }
        else if (command == ManualCommand::WallFollow)
        {
            tunnelFollower.reset();
        }
    }

    manualCommand = command;
    lastManualCommandMs = millis();
}

void setManualSpeed(int16_t speed)
{
    if (speed < MANUAL_SPEED_MIN)
    {
        speed = MANUAL_SPEED_MIN;
    }

    if (speed > MANUAL_SPEED_MAX)
    {
        speed = MANUAL_SPEED_MAX;
    }

    manualSpeed = speed;

    Serial.print("[serial] manual speed=");
    Serial.println(manualSpeed);
}

void handleManualSerialCommand(char command)
{
    switch (command)
    {
        case 'w':
        case 'W':
            setManualCommand(ManualCommand::Forward);
            Serial.println("[manual] forward");
            break;

        case 'b':
        case 'B':
            setManualCommand(ManualCommand::Backward);
            Serial.println("[manual] backward");
            break;

        case 'a':
        case 'A':
            setManualCommand(ManualCommand::Left);
            Serial.println("[manual] left");
            break;

        case 'd':
        case 'D':
            setManualCommand(ManualCommand::Right);
            Serial.println("[manual] right");
            break;

        case 'q':
        case 'Q':
            setManualCommand(ManualCommand::RotateLeft);
            Serial.println("[manual] rotate left");
            break;

        case 'e':
        case 'E':
            setManualCommand(ManualCommand::RotateRight);
            Serial.println("[manual] rotate right");
            break;

        case 'l':
        case 'L':
            setManualCommand(ManualCommand::LineFollow);
            Serial.println("[manual] line follow");
            break;

        case 'g':
        case 'G':
            setManualCommand(ManualCommand::WallFollow);
            Serial.println("[manual] wall follow");
            break;

        case '0':
        case ' ':
            setManualCommand(ManualCommand::Stop);
            stopRobot();
            Serial.println("[manual] stop");
            break;

        default:
            break;
    }
}

bool arenaTimeExpired()
{
    return arenaStartedMs != 0
        && millis() - arenaStartedMs >= ARENA_TIME_LIMIT_MS;
}

void handleRemotePayload(
    const MessageMetadata& metadata,
    const uint8_t* payload,
    size_t length
)
{
    (void)metadata;

    char message[MiniMessenger::kMaxPayloadSize + 1];
    const size_t copyLength =
        length < MiniMessenger::kMaxPayloadSize
            ? length
            : MiniMessenger::kMaxPayloadSize;

    memcpy(message, payload, copyLength);
    message[copyLength] = '\0';

    Serial.print("[mqtt] rx ");
    Serial.println(message);

    if (messageTypeIs(message, "heartbeat"))
    {
        if (tokenTrue(message, "enable="))
        {
            remoteSafetyEnabled = true;
            lastHeartbeatMs = millis();
        }
        else if (tokenFalse(message, "enable="))
        {
            remoteSafetyEnabled = false;
            stopRobot();
        }
        return;
    }

    if (messageTypeIs(message, "start"))
    {
        startRequested = true;
        return;
    }

    if (
        messageTypeIs(message, "stop") ||
        messageTypeIs(message, "emergency") ||
        messageTypeIs(message, "disable")
    )
    {
        remoteSafetyEnabled = false;
        startRequested = false;
        stopRobot();
        return;
    }

    if (messageTypeIs(message, "return") ||
        messageTypeIs(message, "emergency_warning"))
    {
        remoteReturnRequested = true;
        return;
    }

    if (messageTypeIs(message, "stranded"))
    {
        remoteStrandedRequested = true;
        return;
    }

    if (messageTypeIs(message, "revive"))
    {
        remoteReviveRequested = true;
        return;
    }

    if (messageTypeIs(message, "base_reached"))
    {
        baseReachedRequested = true;
        return;
    }

    if (messageTypeIs(message, "openAirlockReply") &&
        tokenTrue(message, "accepted="))
    {
        char airlock[8] = {0};
        if (extractValue(message, "airlock=", airlock, sizeof(airlock)))
        {
            if (strcmp(airlock, "A") == 0 || strcmp(airlock, "a") == 0)
            {
                entryAirlockAccepted = true;
            }
            else if (strcmp(airlock, "B") == 0 || strcmp(airlock, "b") == 0)
            {
                exitAirlockAccepted = true;
            }
        }
        else
        {
            exitAirlockAccepted = true;
            entryAirlockAccepted = true;
        }
        return;
    }

    if (messageTypeIs(message, "isFertileReply") ||
        messageTypeIs(message, "rfid"))
    {
        char tagId[sizeof(pendingTagId)] = {0};
        if (extractValue(message, "tag_id=", tagId, sizeof(tagId)))
        {
            strncpy(pendingTagId, tagId, sizeof(pendingTagId) - 1);
            pendingTagId[sizeof(pendingTagId) - 1] = '\0';
        }

        soilFertile = tokenTrue(message, "fertile=");
        soilAlreadyPlanted = tokenTrue(message, "planted=");
        soilResponseReceived = true;
        return;
    }
}

void pollSerialCommands()
{
    while (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());

        if (command == '\r' || command == '\n')
        {
            continue;
        }

        if (command == '?' || command == 'h' || command == 'H')
        {
            printSerialHelp();
            continue;
        }

        if (command == 'm' || command == 'M')
        {
            enterManualControl();
            Serial.println("[serial] manual control enabled");
            continue;
        }

        if (command == 'p' || command == 'P')
        {
            manualCommand = ManualCommand::Stop;
            stopRobot();
            setState(RunState::Idle);
            Serial.println("[serial] manual control disabled");
            continue;
        }

        if (command == '+' || command == '=')
        {
            setManualSpeed(manualSpeed + MANUAL_SPEED_STEP);
            continue;
        }

        if (command == '-' || command == '_')
        {
            setManualSpeed(manualSpeed - MANUAL_SPEED_STEP);
            continue;
        }

        if (state == RunState::ManualControl &&
            (command == 's' || command == 'S'))
        {
            setManualCommand(ManualCommand::Backward);
            Serial.println("[manual] backward");
            continue;
        }

        if (isManualMovementCommand(command))
        {
            handleManualSerialCommand(command);
            continue;
        }

        if (command == 's' || command == 'S')
        {
            startRequested = true;
            Serial.println("[serial] start requested");
        }
        else if (command == 'r' || command == 'R')
        {
            remoteReturnRequested = true;
            Serial.println("[serial] return requested");
        }
        else if (command == 'x' || command == 'X')
        {
            remoteSafetyEnabled = false;
            stopRobot();
            Serial.println("[serial] safety disabled");
        }
    }
}

void updateInputs()
{
    killSwitch.update();
    reviveButton.update();
    lidarLeft.update();
    lidarRight.update();
    lidarFront.update();

    rfidDetectedThisLoop = false;
    detectedRfidUid[0] = '\0';

    if (rfid.update())
    {
        const String uid = rfid.getUID();
        uid.toCharArray(detectedRfidUid, sizeof(detectedRfidUid));
        rfidDetectedThisLoop = detectedRfidUid[0] != '\0';
    }
}

void updateStatusLed()
{
    if (!safetyAllowed() || state == RunState::Stranded)
    {
        statusLed.showEmergency();
        return;
    }

    if (reviveButton.isPressed())
    {
        statusLed.showButtonPressed();
        return;
    }

    statusLed.showNormal();
}

void handleGlobalRequests()
{
    if (baseReachedRequested)
    {
        baseReachedRequested = false;
        setState(RunState::InsideBase);
        return;
    }

    if (remoteStrandedRequested)
    {
        remoteStrandedRequested = false;

        if (isArenaOrReturnState())
        {
            setState(RunState::Stranded);
            return;
        }
    }

    if (state == RunState::Stranded)
    {
        if (remoteReviveRequested || reviveButton.isPressed())
        {
            remoteReviveRequested = false;
            setState(RunState::ReturnToAirlock);
        }
        return;
    }

    if (remoteReturnRequested)
    {
        remoteReturnRequested = false;

        if (isArenaOrReturnState())
        {
            setState(RunState::ReturnToAirlock);
        }
    }
}

void startMissionIfRequested()
{
    if (!startRequested)
    {
        return;
    }

    if (
        state != RunState::Idle &&
        state != RunState::InsideBase &&
        state != RunState::Finished
    )
    {
        return;
    }

    startRequested = false;
    resetMission();
    setState(RunState::ExitLineToDoor);
}

void beginSoilQueryFromDetectedTag()
{
    const uint32_t now = millis();

    if (!rfidDetectedThisLoop)
    {
        return;
    }

    if (
        strcmp(detectedRfidUid, lastRfidUid) == 0 &&
        now - lastRfidMs < RFID_COOLDOWN_MS
    )
    {
        return;
    }

    strncpy(pendingTagId, detectedRfidUid, sizeof(pendingTagId) - 1);
    pendingTagId[sizeof(pendingTagId) - 1] = '\0';
    strncpy(lastRfidUid, detectedRfidUid, sizeof(lastRfidUid) - 1);
    lastRfidUid[sizeof(lastRfidUid) - 1] = '\0';
    lastRfidMs = now;

    setState(RunState::SoilQuery);
}

void finishPlanting()
{
    if (seedsPlanted < MAX_SEEDS)
    {
        ++seedsPlanted;
    }

    sendSeedPlanted();

    if (seedsPlanted >= MAX_SEEDS)
    {
        setState(RunState::ReturnToAirlock);
    }
    else
    {
        pendingTagId[0] = '\0';
        setState(RunState::GridDrive);
    }
}

void retryOpenAirlockB()
{
    if (millis() - lastServerRequestMs >= SERVER_REQUEST_RETRY_MS)
    {
        sendOpenAirlockB();
    }
}

void retryOpenAirlockA()
{
    if (millis() - lastServerRequestMs >= SERVER_REQUEST_RETRY_MS)
    {
        sendOpenAirlockA();
    }
}

void retrySoilQuery()
{
    if (millis() - lastServerRequestMs >= SERVER_REQUEST_RETRY_MS)
    {
        sendIsFertile();
    }
}

void updateManualControl()
{
    const bool timeoutApplies =
        manualCommand != ManualCommand::LineFollow &&
        manualCommand != ManualCommand::WallFollow;

    if (
        timeoutApplies &&
        manualCommand != ManualCommand::Stop &&
        millis() - lastManualCommandMs > MANUAL_COMMAND_TIMEOUT_MS
    )
    {
        manualCommand = ManualCommand::Stop;
        stopRobot();
        Serial.println("[manual] command timeout");
        return;
    }

    switch (manualCommand)
    {
        case ManualCommand::Stop:
            stopRobot();
            break;

        case ManualCommand::Forward:
            robot.forward(manualSpeed);
            break;

        case ManualCommand::Backward:
            robot.backward(manualSpeed);
            break;

        case ManualCommand::Left:
            robot.left(manualSpeed);
            break;

        case ManualCommand::Right:
            robot.right(manualSpeed);
            break;

        case ManualCommand::RotateLeft:
            robot.rotate_left(manualSpeed);
            break;

        case ManualCommand::RotateRight:
            robot.rotate_right(manualSpeed);
            break;

        case ManualCommand::LineFollow:
            lineFollower.update(manualSpeed);
            break;

        case ManualCommand::WallFollow:
            tunnelFollower.update(manualSpeed);
            break;
    }
}

void updateState()
{
    startMissionIfRequested();

    switch (state)
    {
        case RunState::Idle:
        case RunState::Finished:
            stopRobot();
            break;

        case RunState::ExitLineToDoor:
            lineFollower.update(LINE_SPEED);
            if (doorNearStable())
            {
                setState(RunState::ExitRequest);
            }
            break;

        case RunState::ExitRequest:
            stopRobot();
            if (exitAirlockAccepted)
            {
                setState(RunState::ExitWaitDoor);
            }
            else
            {
                retryOpenAirlockB();
            }
            break;

        case RunState::ExitWaitDoor:
            stopRobot();
            if (doorOpenStable())
            {
                setState(RunState::ExitTraverseTunnel);
            }
            break;

        case RunState::ExitTraverseTunnel:
            tunnelFollower.update(TUNNEL_SPEED);
            if (stateElapsed(TUNNEL_TRAVERSE_MS))
            {
                arenaStartedMs = millis();
                setState(RunState::GridDrive);
            }
            break;

        case RunState::GridDrive:
            if (seedsPlanted >= MAX_SEEDS || arenaTimeExpired())
            {
                setState(RunState::ReturnToAirlock);
                break;
            }

            beginSoilQueryFromDetectedTag();
            if (state != RunState::GridDrive)
            {
                break;
            }

            if (frontObstacleBlocked())
            {
                robot.rotate_right(AVOID_TURN_SPEED);
            }
            else
            {
                lineFollower.update(GRID_SPEED);
            }
            break;

        case RunState::SoilQuery:
            stopRobot();
            if (soilResponseReceived)
            {
                if (soilFertile && !soilAlreadyPlanted && seedsPlanted < MAX_SEEDS)
                {
                    setState(RunState::AlignSearch);
                }
                else
                {
                    pendingTagId[0] = '\0';
                    setState(RunState::GridDrive);
                }
            }
            else if (stateElapsed(SOIL_QUERY_TIMEOUT_MS))
            {
                pendingTagId[0] = '\0';
                setState(RunState::GridDrive);
            }
            else
            {
                retrySoilQuery();
            }
            break;

        case RunState::AlignSearch:
            stopRobot();
            if (stateElapsed(ALIGN_SEARCH_MS))
            {
                setState(RunState::FineAdjust);
            }
            break;

        case RunState::FineAdjust:
            robot.forward(FINE_ADJUST_SPEED);
            if (stateElapsed(FINE_ADJUST_MS))
            {
                setState(RunState::PlantOpen);
            }
            break;

        case RunState::PlantOpen:
            stopRobot();
            if (stateElapsed(HOPPER_OPEN_MS))
            {
                setState(RunState::PlantDrop);
            }
            break;

        case RunState::PlantDrop:
            stopRobot();
            if (stateElapsed(DROP_SEED_MS))
            {
                setState(RunState::PlantVerify);
            }
            break;

        case RunState::PlantVerify:
            stopRobot();
            if (stateElapsed(PLANT_VERIFY_MS))
            {
                finishPlanting();
            }
            break;

        case RunState::ReturnToAirlock:
            if (doorNearStable())
            {
                setState(RunState::EntryRequest);
            }
            else if (frontObstacleBlocked())
            {
                robot.rotate_right(AVOID_TURN_SPEED);
            }
            else
            {
                lineFollower.update(RETURN_SPEED);
            }
            break;

        case RunState::EntryRequest:
            stopRobot();
            if (entryAirlockAccepted)
            {
                setState(RunState::EntryWaitDoor);
            }
            else
            {
                retryOpenAirlockA();
            }
            break;

        case RunState::EntryWaitDoor:
            stopRobot();
            if (doorOpenStable() ||
                (entryAirlockAccepted && stateElapsed(ENTRY_ACCEPT_FALLBACK_MS)))
            {
                setState(RunState::EntryTraverseTunnel);
            }
            break;

        case RunState::EntryTraverseTunnel:
            tunnelFollower.update(TUNNEL_SPEED);
            if (stateElapsed(ENTRY_TUNNEL_TRAVERSE_MS))
            {
                setState(RunState::InsideBase);
            }
            break;

        case RunState::InsideBase:
            stopRobot();
            break;

        case RunState::Stranded:
            stopRobot();
            break;

        case RunState::ManualControl:
            updateManualControl();
            break;
    }
}
} // namespace

void systemSetup()
{
    Serial.begin(SERIAL_BAUD);

    lineSensors.begin();
    lidarLeft.begin();
    lidarRight.begin();
    lidarFront.begin();
    rfid.begin();
    killSwitch.begin();
    reviveButton.begin();
    statusLed.begin();
    hopperServo.attach(SERVO_PIN);
    closeHopper();

    motorReady = robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    if (motorReady)
    {
        encoderControlReady = robot.begin_encoder_speed_control();
        resetLineControl();
    }
    stopRobot();

    messenger.onMessage(handleRemotePayload);
    messenger.begin(
        WIFI_SSID,
        WIFI_PASSWORD,
        BROKER_HOST,
        BROKER_PORT,
        GROUP_ID,
        BOARD_ID
    );

    stateStartedMs = millis();
    lastRegisterMs = millis() - WIFI_REGISTER_INTERVAL_MS;

    Serial.println("Competition system ready.");
    Serial.println(motorReady ? "Motoron init OK." : "Motoron init FAILED.");
    Serial.println(
        encoderControlReady
            ? "Encoder speed control OK."
            : "Encoder speed control FAILED."
    );
    if (!encoderControlReady)
    {
        Serial.println("Line follow is disabled until encoder control works.");
    }
    printSerialHelp();
}

void systemLoop()
{
    messenger.loop();
    pollSerialCommands();
    sendRegister();
    updateInputs();

    if (!safetyAllowed())
    {
        manualCommand = ManualCommand::Stop;
        stopRobot();
        updateStatusLed();
        sendStatus();
        return;
    }

    handleGlobalRequests();
    updateState();
    robot.update_encoder_speed_control();
    updateStatusLed();
    sendStatus();
}

#else

void systemSetup()
{
    Serial.begin(115200);
}

void systemLoop()
{
    delay(10);
}

#endif
