#pragma once

#if defined(CORE_CM7)

#include <Arduino.h>

namespace RobotApp
{
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
