#include "system.h"

#include <Arduino.h>

#if defined(CORE_CM7)

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "IRConfig.h"
#include "IRSensor.h"
#include "KillSwitch.h"
#include "LED.h"
#include "LineFollower.h"
#include "LidarSensor.h"
#include "MiniMessenger.h"
#include "MotorConfig.h"
#include "MotoronDrive.h"
#include "Planter.h"
#include "RFIDHandler.h"
#include "ReviveButton.h"
#include "WallFollower.h"
#include "WiFiHandlerConfig.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t STATUS_INTERVAL_MS = 500;
constexpr uint32_t LIDAR_FRESH_MS = 200;

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
constexpr uint32_t ALIGN_SEARCH_TIMEOUT_MS = 2000;
constexpr uint16_t ALIGN_IR_MIN_VALUE = 10;
constexpr uint16_t ALIGN_IR_MAX_VALUE = 30;
constexpr uint8_t ALIGN_IR_SENSOR_INDEX = IR_SENSOR_COUNT / 2;
constexpr float FINE_ADJUST_DISTANCE_CM = 2.0f;
constexpr uint32_t FINE_ADJUST_TIMEOUT_MS = 1500;
constexpr uint32_t PLANT_VERIFY_MS = 300;
constexpr uint32_t RFID_COOLDOWN_MS = 2000;

constexpr uint8_t MAX_SEEDS = 5;
constexpr uint8_t GRID_WIDTH = 9;
constexpr uint8_t GRID_HEIGHT = 9;
constexpr uint8_t GRID_CELL_COUNT = GRID_WIDTH * GRID_HEIGHT;
constexpr uint8_t GRID_AIRLOCK_ROW = 1;
constexpr uint8_t GRID_ENTRY_COLUMN = 3;
constexpr uint8_t GRID_EXIT_COLUMN = 7;
constexpr uint8_t GRID_LINE_ROW_LIMIT = 4;

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
    PlanterTest,
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

struct GridPoint
{
    int8_t x;
    int8_t y;
};

struct GridCell
{
    int8_t x;
    int8_t y;
    bool known;
    bool fertile;
    bool planted;
    bool blocked;
    char tagId[40];
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
Planter planter(robot);

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
bool fineAdjustMoveStarted = false;

uint8_t seedsPlanted = 0;

GridCell arenaGrid[GRID_HEIGHT][GRID_WIDTH];
GridPoint currentGridPosition = {-1, -1};
GridPoint routeTargetPosition = {-1, -1};
GridPoint plannedRoute[GRID_CELL_COUNT];
uint8_t plannedRouteLength = 0;
bool plannedRouteValid = false;

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
        case RunState::PlanterTest:
            return "PlanterTest";
        case RunState::Finished:
            return "Finished";
    }

    return "Unknown";
}

void stopRobot()
{
    robot.stop_all();
}

void stopDriveWheels()
{
    robot.set_all(0, 0, 0, 0);
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
    Serial.println("  O/C/T - run planter 180-degree cycle");
    Serial.println("  0 or Space - manual stop");
    Serial.println("  + / - - manual speed up/down");
    Serial.println("  ? - print this help");
}

void closeHopper()
{
    planter.stop();
}

void openHopper()
{
    planter.startCycle();
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

bool plantIrAligned(uint16_t& value)
{
    lineSensors.update();
    value = lineSensors.getValue(ALIGN_IR_SENSOR_INDEX);
    return value >= ALIGN_IR_MIN_VALUE && value <= ALIGN_IR_MAX_VALUE;
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

bool extractIntValue(const char* text, const char* key, int& output)
{
    char value[12] = {0};
    if (!extractValue(text, key, value, sizeof(value)))
    {
        return false;
    }

    output = atoi(value);
    return true;
}

bool gridCoordinateToIndex(int x, int y, uint8_t& indexX, uint8_t& indexY)
{
    if (
        x >= 1 && x <= static_cast<int>(GRID_WIDTH) &&
        y >= 1 && y <= static_cast<int>(GRID_HEIGHT)
    )
    {
        indexX = static_cast<uint8_t>(x - 1);
        indexY = static_cast<uint8_t>(y - 1);
        return true;
    }

    if (
        x >= 0 && x < static_cast<int>(GRID_WIDTH) &&
        y >= 0 && y < static_cast<int>(GRID_HEIGHT)
    )
    {
        indexX = static_cast<uint8_t>(x);
        indexY = static_cast<uint8_t>(y);
        return true;
    }

    return false;
}

bool gridPointToIndex(GridPoint point, uint8_t& indexX, uint8_t& indexY)
{
    return gridCoordinateToIndex(point.x, point.y, indexX, indexY);
}

bool gridIndexHasLine(uint8_t indexY)
{
    return indexY < GRID_LINE_ROW_LIMIT;
}

void resetPlannedRoute()
{
    routeTargetPosition = {-1, -1};
    plannedRouteLength = 0;
    plannedRouteValid = false;
}

void initializeGridMap()
{
    for (uint8_t y = 0; y < GRID_HEIGHT; ++y)
    {
        for (uint8_t x = 0; x < GRID_WIDTH; ++x)
        {
            arenaGrid[y][x].x = static_cast<int8_t>(x + 1);
            arenaGrid[y][x].y = static_cast<int8_t>(y + 1);
            arenaGrid[y][x].known = false;
            arenaGrid[y][x].fertile = false;
            arenaGrid[y][x].planted = false;
            arenaGrid[y][x].blocked = false;
            arenaGrid[y][x].tagId[0] = '\0';
        }
    }

    uint8_t entryX = 0;
    uint8_t entryY = 0;
    uint8_t exitX = 0;
    uint8_t exitY = 0;
    if (
        gridCoordinateToIndex(GRID_ENTRY_COLUMN, GRID_AIRLOCK_ROW, entryX, entryY) &&
        gridCoordinateToIndex(GRID_EXIT_COLUMN, GRID_AIRLOCK_ROW, exitX, exitY)
    )
    {
        arenaGrid[entryY][entryX].known = true;
        arenaGrid[exitY][exitX].known = true;
    }

    currentGridPosition = {-1, -1};
    resetPlannedRoute();
}

void printPlannedRoute()
{
    if (!plannedRouteValid || plannedRouteLength == 0)
    {
        Serial.println("[nav] no route");
        return;
    }

    Serial.print("[nav] route ");
    Serial.print(plannedRouteLength);
    Serial.print(" cells:");

    for (uint8_t i = 0; i < plannedRouteLength; ++i)
    {
        Serial.print(" (");
        Serial.print(plannedRoute[i].x);
        Serial.print(",");
        Serial.print(plannedRoute[i].y);
        Serial.print(")");
    }

    Serial.println();
}

bool calculateRoute(GridPoint start, GridPoint target)
{
    uint8_t startX = 0;
    uint8_t startY = 0;
    uint8_t targetX = 0;
    uint8_t targetY = 0;

    resetPlannedRoute();

    if (
        !gridPointToIndex(start, startX, startY) ||
        !gridPointToIndex(target, targetX, targetY) ||
        arenaGrid[startY][startX].blocked ||
        arenaGrid[targetY][targetX].blocked ||
        !gridIndexHasLine(startY) ||
        !gridIndexHasLine(targetY)
    )
    {
        return false;
    }

    bool visited[GRID_HEIGHT][GRID_WIDTH] = {};
    int8_t previousX[GRID_HEIGHT][GRID_WIDTH];
    int8_t previousY[GRID_HEIGHT][GRID_WIDTH];
    GridPoint queue[GRID_CELL_COUNT];
    uint8_t head = 0;
    uint8_t tail = 0;

    for (uint8_t y = 0; y < GRID_HEIGHT; ++y)
    {
        for (uint8_t x = 0; x < GRID_WIDTH; ++x)
        {
            previousX[y][x] = -1;
            previousY[y][x] = -1;
        }
    }

    visited[startY][startX] = true;
    queue[tail++] = {static_cast<int8_t>(startX), static_cast<int8_t>(startY)};

    while (head < tail)
    {
        const GridPoint current = queue[head++];
        if (current.x == static_cast<int8_t>(targetX) &&
            current.y == static_cast<int8_t>(targetY))
        {
            break;
        }

        const int8_t offsetsX[4] = {1, -1, 0, 0};
        const int8_t offsetsY[4] = {0, 0, 1, -1};

        for (uint8_t i = 0; i < 4; ++i)
        {
            const int8_t nextX = current.x + offsetsX[i];
            const int8_t nextY = current.y + offsetsY[i];

            if (
                nextX < 0 ||
                nextY < 0 ||
                nextX >= static_cast<int8_t>(GRID_WIDTH) ||
                nextY >= static_cast<int8_t>(GRID_HEIGHT)
            )
            {
                continue;
            }

            if (
                visited[nextY][nextX] ||
                arenaGrid[nextY][nextX].blocked ||
                !gridIndexHasLine(nextY)
            )
            {
                continue;
            }

            visited[nextY][nextX] = true;
            previousX[nextY][nextX] = current.x;
            previousY[nextY][nextX] = current.y;
            queue[tail++] = {nextX, nextY};
        }
    }

    if (!visited[targetY][targetX])
    {
        return false;
    }

    GridPoint reverseRoute[GRID_CELL_COUNT];
    uint8_t reverseLength = 0;
    int8_t cursorX = static_cast<int8_t>(targetX);
    int8_t cursorY = static_cast<int8_t>(targetY);

    while (cursorX >= 0 && cursorY >= 0 && reverseLength < GRID_CELL_COUNT)
    {
        reverseRoute[reverseLength++] = {
            arenaGrid[cursorY][cursorX].x,
            arenaGrid[cursorY][cursorX].y
        };

        if (cursorX == static_cast<int8_t>(startX) &&
            cursorY == static_cast<int8_t>(startY))
        {
            break;
        }

        const int8_t nextCursorX = previousX[cursorY][cursorX];
        const int8_t nextCursorY = previousY[cursorY][cursorX];
        cursorX = nextCursorX;
        cursorY = nextCursorY;
    }

    for (uint8_t i = 0; i < reverseLength; ++i)
    {
        plannedRoute[i] = reverseRoute[reverseLength - 1 - i];
    }

    routeTargetPosition = target;
    plannedRouteLength = reverseLength;
    plannedRouteValid = true;
    return true;
}

bool findNearestPlantableCell(GridPoint& target)
{
    if (currentGridPosition.x < 0 || currentGridPosition.y < 0)
    {
        return false;
    }

    uint8_t currentX = 0;
    uint8_t currentY = 0;
    if (!gridPointToIndex(currentGridPosition, currentX, currentY))
    {
        return false;
    }

    bool found = false;
    uint8_t bestDistance = GRID_CELL_COUNT;

    for (uint8_t y = 0; y < GRID_HEIGHT; ++y)
    {
        for (uint8_t x = 0; x < GRID_WIDTH; ++x)
        {
            GridCell& cell = arenaGrid[y][x];
            if (
                !cell.known ||
                !cell.fertile ||
                cell.planted ||
                cell.blocked ||
                !gridIndexHasLine(y)
            )
            {
                continue;
            }

            const uint8_t distance =
                abs(static_cast<int>(x) - static_cast<int>(currentX)) +
                abs(static_cast<int>(y) - static_cast<int>(currentY));

            if (!found || distance < bestDistance)
            {
                found = true;
                bestDistance = distance;
                target = {cell.x, cell.y};
            }
        }
    }

    return found;
}

void calculateRouteToNearestPlantableCell()
{
    GridPoint target = {-1, -1};
    if (!findNearestPlantableCell(target))
    {
        resetPlannedRoute();
        Serial.println("[nav] no known plantable target");
        return;
    }

    if (calculateRoute(currentGridPosition, target))
    {
        printPlannedRoute();
    }
    else
    {
        Serial.println("[nav] target exists but no line-follow route was found");
    }
}

bool updateGridCellFromMessage(const char* message)
{
    int coordinateX = 0;
    int coordinateY = 0;
    if (
        !extractIntValue(message, "x=", coordinateX) ||
        !extractIntValue(message, "y=", coordinateY)
    )
    {
        return false;
    }

    uint8_t indexX = 0;
    uint8_t indexY = 0;
    if (!gridCoordinateToIndex(coordinateX, coordinateY, indexX, indexY))
    {
        Serial.println("[nav] ignored RFID reply with out-of-range grid coordinate");
        return false;
    }

    GridCell& cell = arenaGrid[indexY][indexX];
    cell.x = static_cast<int8_t>(coordinateX);
    cell.y = static_cast<int8_t>(coordinateY);
    cell.known = true;
    cell.fertile = tokenTrue(message, "fertile=");
    cell.planted = tokenTrue(message, "planted=");
    cell.blocked =
        tokenTrue(message, "blocked=") ||
        tokenTrue(message, "obstacle=");

    if (pendingTagId[0] != '\0')
    {
        strncpy(cell.tagId, pendingTagId, sizeof(cell.tagId) - 1);
        cell.tagId[sizeof(cell.tagId) - 1] = '\0';
    }

    currentGridPosition = {cell.x, cell.y};

    Serial.print("[nav] grid cell ");
    Serial.print(cell.x);
    Serial.print(",");
    Serial.print(cell.y);
    Serial.print(" fertile=");
    Serial.print(cell.fertile ? "true" : "false");
    Serial.print(" planted=");
    Serial.print(cell.planted ? "true" : "false");
    Serial.print(" blocked=");
    Serial.println(cell.blocked ? "true" : "false");

    calculateRouteToNearestPlantableCell();
    return true;
}

void markCurrentGridCellPlanted()
{
    uint8_t indexX = 0;
    uint8_t indexY = 0;
    if (!gridPointToIndex(currentGridPosition, indexX, indexY))
    {
        return;
    }

    GridCell& cell = arenaGrid[indexY][indexX];
    cell.known = true;
    cell.fertile = true;
    cell.planted = true;

    if (pendingTagId[0] != '\0')
    {
        strncpy(cell.tagId, pendingTagId, sizeof(cell.tagId) - 1);
        cell.tagId[sizeof(cell.tagId) - 1] = '\0';
    }

    calculateRouteToNearestPlantableCell();
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
    fineAdjustMoveStarted = false;
    pendingTagId[0] = '\0';
    lastRfidUid[0] = '\0';
    initializeGridMap();
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
        case RunState::PlanterTest:
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
    else if (state == RunState::FineAdjust)
    {
        fineAdjustMoveStarted = robot.start_forward_cm(
            FINE_ADJUST_DISTANCE_CM,
            FINE_ADJUST_SPEED,
            FINE_ADJUST_TIMEOUT_MS
        );

        Serial.print("[align] forward ");
        Serial.print(FINE_ADJUST_DISTANCE_CM);
        Serial.println("cm");

        if (!fineAdjustMoveStarted)
        {
            Serial.println("[align] failed to start fine adjust move");
        }
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
    else if (state == RunState::PlanterTest)
    {
        manualCommand = ManualCommand::Stop;
        openHopper();
        Serial.println("[planter] test open");
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

bool manualControlAllowed()
{
    return motorReady && killSwitch.isSafe();
}

bool planterTestAllowed()
{
    return motorReady && killSwitch.isSafe();
}

bool localControlAllowed()
{
    if (state == RunState::ManualControl)
    {
        return manualControlAllowed();
    }

    if (state == RunState::PlanterTest)
    {
        return planterTestAllowed();
    }

    return false;
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

void startPlanterTest()
{
    startRequested = false;
    remoteReturnRequested = false;
    manualCommand = ManualCommand::Stop;
    stopRobot();
    setState(RunState::PlanterTest);
}

void manualRunPlanterCycle()
{
    startPlanterTest();
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
        updateGridCellFromMessage(message);
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
            closeHopper();
            setState(RunState::Idle);
            Serial.println("[serial] manual control disabled");
            continue;
        }

        if (command == 'o' || command == 'O')
        {
            manualRunPlanterCycle();
            continue;
        }

        if (command == 'c' || command == 'C')
        {
            manualRunPlanterCycle();
            continue;
        }

        if (command == 't' || command == 'T')
        {
            startPlanterTest();
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
    if ((!safetyAllowed() && !localControlAllowed()) || state == RunState::Stranded)
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

    markCurrentGridCellPlanted();
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

void updatePlanterTest()
{
    stopDriveWheels();

    if (planter.update())
    {
        Serial.print("[planter] test complete counts=");
        Serial.println(planter.position());
        setState(RunState::ManualControl);
    }
    else if (planter.hasTimedOut())
    {
        Serial.print("[planter] test timeout counts=");
        Serial.println(planter.position());
        setState(RunState::ManualControl);
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
            {
                uint16_t alignIrValue = 0;
                if (plantIrAligned(alignIrValue))
                {
                    Serial.print("[align] IR");
                    Serial.print(ALIGN_IR_SENSOR_INDEX);
                    Serial.print("=");
                    Serial.println(alignIrValue);
                    setState(RunState::FineAdjust);
                }
                else if (stateElapsed(ALIGN_SEARCH_TIMEOUT_MS))
                {
                    Serial.println("[align] IR alignment timeout");
                    pendingTagId[0] = '\0';
                    setState(RunState::GridDrive);
                }
            }
            break;

        case RunState::FineAdjust:
            if (!fineAdjustMoveStarted)
            {
                pendingTagId[0] = '\0';
                setState(RunState::GridDrive);
                break;
            }

            if (robot.update_distance_move())
            {
                setState(RunState::PlantOpen);
            }
            else if (!robot.distance_move_active())
            {
                Serial.println("[align] fine adjust move failed");
                pendingTagId[0] = '\0';
                setState(RunState::GridDrive);
            }
            break;

        case RunState::PlantOpen:
            stopDriveWheels();
            if (planter.update())
            {
                setState(RunState::PlantVerify);
            }
            else if (planter.hasTimedOut())
            {
                Serial.print("[planter] planting timeout counts=");
                Serial.println(planter.position());
                setState(RunState::GridDrive);
            }
            break;

        case RunState::PlantDrop:
            stopDriveWheels();
            if (planter.update())
            {
                setState(RunState::PlantVerify);
            }
            else if (planter.hasTimedOut())
            {
                Serial.print("[planter] planting timeout counts=");
                Serial.println(planter.position());
                setState(RunState::GridDrive);
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

        case RunState::PlanterTest:
            updatePlanterTest();
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
    initializeGridMap();

    motorReady = robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    if (motorReady)
    {
        encoderControlReady = robot.begin_encoder_speed_control();
        resetLineControl();
    }
    stopRobot();
    planter.begin();

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

    const bool autonomousAllowed = safetyAllowed();
    const bool localAllowed = localControlAllowed();

    if (!autonomousAllowed && !localAllowed)
    {
        manualCommand = ManualCommand::Stop;
        stopRobot();
        updateStatusLed();
        sendStatus();
        return;
    }

    if (autonomousAllowed)
    {
        handleGlobalRequests();
    }

    updateState();
    if (encoderControlReady)
    {
        robot.update_encoder_speed_control();
    }
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
