#pragma once

#if defined(CORE_CM7)

#include <Arduino.h>

namespace RobotApp
{
enum class AutomaticMotionPhase : uint8_t
{
    Idle,
    ExitLineToDoor,
    ExitWaitForDoor,
    ExitTraverseTunnel,
    ReturnToAirlock,
    EntryWaitForDoor,
    EntryTraverseTunnel
};

void updateAutomaticEventContext(
    AutomaticMotionPhase phase,
    int16_t lidarDistanceCm,
    bool lidarValid,
    bool lineVisible
);
void notifyExitAirlockAccepted();
void notifyEntryAirlockAccepted();
void notifyRobotStranded();
bool exitClearanceReceivedAutomatically();
bool exitDoorDetectedAutomatically();
bool exitDoorOpenedAutomatically();
bool mainArenaReachedAutomatically();
bool entryAirlockReachedAutomatically();
bool entryDoorOpenedAutomatically();
bool baseReachedAutomatically();
bool robotIsStrandedAutomatically();
bool lookupRfidTag(
    const String& uid,
    char* coordinate,
    size_t coordinateSize,
    bool& fertile
);
} // namespace RobotApp

#endif
