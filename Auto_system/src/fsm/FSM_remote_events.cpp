#if defined(CORE_CM7)

#include "FSM_remote_events.h"

#include <stdio.h>
#include <string.h>

namespace RobotApp
{
void RemoteMessageParser::copyPayload(const uint8_t* payload, size_t length)
{
    const size_t copyLength =
        (length < MiniMessenger::kMaxPayloadSize)
        ?
        length
        :
        MiniMessenger::kMaxPayloadSize;

    memcpy(lastMessage_, payload, copyLength);
    lastMessage_[copyLength] = '\0';
}

bool RemoteMessageParser::messageContains(const char* token) const
{
    return strstr(lastMessage_, token) != nullptr;
}

bool RemoteMessageParser::messageFieldEquals(
    const char* key,
    const char* value
) const
{
    char token[48];
    snprintf(token, sizeof(token), "%s=%s", key, value);
    return messageContains(token);
}

void RemoteMessageParser::parseInto(RemoteEvents& events) const
{
    if (messageFieldEquals("type", "start"))
    {
        events.start = true;
    }
    else if (messageFieldEquals("type", "clearance"))
    {
        events.exitClearance = true;
    }
    else if (messageFieldEquals("type", "exit_door_detected"))
    {
        events.exitDoorDetected = true;
    }
    else if (messageFieldEquals("type", "exit_door_opened"))
    {
        events.exitDoorOpened = true;
    }
    else if (messageFieldEquals("type", "arena_reached"))
    {
        events.mainArenaReached = true;
    }
    else if (messageFieldEquals("type", "return"))
    {
        events.emergencyReturn = true;
    }
    else if (messageFieldEquals("type", "entry_airlock_reached"))
    {
        events.entryAirlockReached = true;
    }
    else if (messageFieldEquals("type", "entry_door_opened"))
    {
        events.entryDoorOpened = true;
    }
    else if (messageFieldEquals("type", "base_reached"))
    {
        events.baseReached = true;
    }
    else if (messageFieldEquals("type", "stranded"))
    {
        events.stranded = true;
    }
    else if (messageFieldEquals("type", "revive"))
    {
        events.revive = true;
    }
    else if (messageFieldEquals("type", "rfid"))
    {
        events.rfid = true;
        events.rfidFertile =
            messageFieldEquals("fertile", "1") ||
            messageFieldEquals("fertile", "true");
        copyCoordinateFromMessage(
            events.rfidCoordinate,
            sizeof(events.rfidCoordinate)
        );
    }
}

const char* RemoteMessageParser::message() const
{
    return lastMessage_;
}

void RemoteMessageParser::copyCoordinateFromMessage(
    char* destination,
    size_t destinationSize
) const
{
    destination[0] = '\0';

    const char* coordinateStart = strstr(lastMessage_, "coordinate=");
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
} // namespace RobotApp

#endif
