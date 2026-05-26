#if defined(CORE_CM7)

#include "FSM_auto_events.h"

#include "../navigation/NavigationRuntime.h"

namespace RobotApp
{
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
    return RobotNavigation::navigator().observeRfidTag(
        uid,
        coordinate,
        coordinateSize,
        fertile
    );
}
} // namespace RobotApp

#endif
