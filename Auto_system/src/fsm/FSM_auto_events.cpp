#if defined(CORE_CM7)

#include "FSM_auto_events.h"

#include "../navigation/NavigationRuntime.h"

namespace RobotApp
{
namespace
{
constexpr int16_t DOOR_DETECTED_DISTANCE_CM = 18;
constexpr int16_t DOOR_OPEN_DISTANCE_CM = 45;
constexpr uint32_t DOOR_STABLE_MS = 180;
constexpr uint32_t TUNNEL_TRAVERSE_MS = 4500;
constexpr uint32_t TUNNEL_TRAVERSE_FALLBACK_MS = 7500;
constexpr uint32_t LINE_CONFIRM_MS = 250;

AutomaticMotionPhase currentPhase = AutomaticMotionPhase::Idle;
int16_t currentLidarDistanceCm = -1;
bool currentLidarValid = false;
bool currentLineVisible = false;
bool exitAirlockAccepted = false;
bool entryAirlockAccepted = false;
bool strandedDetected = false;
uint32_t phaseStartedMs = 0;
uint32_t doorNearSinceMs = 0;
uint32_t doorClearSinceMs = 0;
uint32_t lineVisibleSinceMs = 0;

bool consumeFlag(bool& flag)
{
    const bool wasSet = flag;
    flag = false;
    return wasSet;
}

bool phaseElapsed(uint32_t durationMs)
{
    return millis() - phaseStartedMs >= durationMs;
}

bool stableSince(bool condition, uint32_t& sinceMs, uint32_t durationMs)
{
    if (!condition)
    {
        sinceMs = 0;
        return false;
    }

    if (sinceMs == 0)
    {
        sinceMs = millis();
        return false;
    }

    return millis() - sinceMs >= durationMs;
}

bool doorIsNear()
{
    return
        currentLidarValid &&
        currentLidarDistanceCm > 0 &&
        currentLidarDistanceCm <= DOOR_DETECTED_DISTANCE_CM;
}

bool doorPathIsClear()
{
    return
        currentLidarValid &&
        currentLidarDistanceCm >= DOOR_OPEN_DISTANCE_CM;
}
} // namespace

void updateAutomaticEventContext(
    AutomaticMotionPhase phase,
    int16_t lidarDistanceCm,
    bool lidarValid,
    bool lineVisible
)
{
    if (phase != currentPhase)
    {
        currentPhase = phase;
        phaseStartedMs = millis();
        doorNearSinceMs = 0;
        doorClearSinceMs = 0;
        lineVisibleSinceMs = 0;
    }

    currentLidarDistanceCm = lidarDistanceCm;
    currentLidarValid = lidarValid;
    currentLineVisible = lineVisible;
}

void notifyExitAirlockAccepted()
{
    exitAirlockAccepted = true;
}

void notifyEntryAirlockAccepted()
{
    entryAirlockAccepted = true;
}

void notifyRobotStranded()
{
    strandedDetected = true;
}

bool exitClearanceReceivedAutomatically()
{
    return consumeFlag(exitAirlockAccepted);
}

bool exitDoorDetectedAutomatically()
{
    return
        currentPhase == AutomaticMotionPhase::ExitLineToDoor &&
        stableSince(doorIsNear(), doorNearSinceMs, DOOR_STABLE_MS);
}

bool exitDoorOpenedAutomatically()
{
    return
        currentPhase == AutomaticMotionPhase::ExitWaitForDoor &&
        stableSince(doorPathIsClear(), doorClearSinceMs, DOOR_STABLE_MS);
}

bool mainArenaReachedAutomatically()
{
    return
        currentPhase == AutomaticMotionPhase::ExitTraverseTunnel &&
        phaseElapsed(TUNNEL_TRAVERSE_MS);
}

bool entryAirlockReachedAutomatically()
{
    return
        currentPhase == AutomaticMotionPhase::ReturnToAirlock &&
        stableSince(doorIsNear(), doorNearSinceMs, DOOR_STABLE_MS);
}

bool entryDoorOpenedAutomatically()
{
    return
        consumeFlag(entryAirlockAccepted) ||
        (
            currentPhase == AutomaticMotionPhase::EntryWaitForDoor &&
            stableSince(doorPathIsClear(), doorClearSinceMs, DOOR_STABLE_MS)
        );
}

bool baseReachedAutomatically()
{
    return
        currentPhase == AutomaticMotionPhase::EntryTraverseTunnel &&
        (
            (
                phaseElapsed(TUNNEL_TRAVERSE_MS) &&
                stableSince(currentLineVisible, lineVisibleSinceMs, LINE_CONFIRM_MS)
            ) ||
            phaseElapsed(TUNNEL_TRAVERSE_FALLBACK_MS)
        );
}

bool robotIsStrandedAutomatically()
{
    return consumeFlag(strandedDetected);
}

bool lookupRfidTag(
    const String& uid,
    char* coordinate,
    size_t coordinateSize,
    bool& fertile
)
{
    return RobotNavigation::navigator().observeRfidTag(
        uid,
        coordinate,
        coordinateSize,
        fertile
    );
}
} // namespace RobotApp

#endif
