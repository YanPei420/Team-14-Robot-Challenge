#if defined(CORE_CM7)

#include "FSM_remote_events.h"

#include <stdio.h>
#include <stdlib.h>
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

bool RemoteMessageParser::messageFieldIsTruthy(const char* key) const
{
    return
        messageFieldEquals(key, "1") ||
        messageFieldEquals(key, "true") ||
        messageFieldEquals(key, "TRUE") ||
        messageFieldEquals(key, "yes") ||
        messageFieldEquals(key, "accepted");
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
    else if (messageFieldEquals("type", "emergency_warning"))
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
    else if (messageFieldEquals("type", "disable"))
    {
        events.stranded = true;
    }
    else if (messageFieldEquals("type", "openAirlockReply"))
    {
        events.airlockReply = true;
        events.airlockAccepted = messageFieldIsTruthy("accepted");
    }
    else if (messageFieldEquals("type", "isFertileReply"))
    {
        events.soilReply = true;
        events.rfidFertile = messageFieldIsTruthy("fertile");
        events.rfidAlreadyPlanted = messageFieldIsTruthy("planted");

        copyFieldFromMessage(
            "tag_id",
            events.rfidTagId,
            sizeof(events.rfidTagId)
        );

        if (!copyCoordinateFromServerFields(
                events.rfidCoordinate,
                sizeof(events.rfidCoordinate)
            ))
        {
            copyCoordinateFromMessage(
                events.rfidCoordinate,
                sizeof(events.rfidCoordinate)
            );
        }
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

void RemoteMessageParser::copyFieldFromMessage(
    const char* key,
    char* destination,
    size_t destinationSize
) const
{
    if (destination == nullptr || destinationSize == 0)
    {
        return;
    }

    destination[0] = '\0';

    char prefix[24];
    snprintf(prefix, sizeof(prefix), "%s=", key);

    const char* valueStart = strstr(lastMessage_, prefix);
    if (valueStart == nullptr)
    {
        return;
    }

    valueStart += strlen(prefix);

    size_t i = 0;
    while (
        valueStart[i] != '\0' &&
        valueStart[i] != ' ' &&
        i + 1 < destinationSize
    )
    {
        destination[i] = valueStart[i];
        ++i;
    }

    destination[i] = '\0';
}

bool RemoteMessageParser::readIntegerField(const char* key, int& value) const
{
    char field[12];
    copyFieldFromMessage(key, field, sizeof(field));

    if (field[0] == '\0')
    {
        return false;
    }

    value = atoi(field);
    return true;
}

bool RemoteMessageParser::copyCoordinateFromServerFields(
    char* destination,
    size_t destinationSize
) const
{
    if (destination == nullptr || destinationSize < 3)
    {
        return false;
    }

    int x = 0;
    int y = 0;
    if (!readIntegerField("x", x) || !readIntegerField("y", y))
    {
        return false;
    }

    if (x >= 1 && x <= 9)
    {
        --x;
    }

    if (y >= 1 && y <= 9)
    {
        --y;
    }

    if (x < 0 || x > 8 || y < 0 || y > 8)
    {
        return false;
    }

    snprintf(
        destination,
        destinationSize,
        "%c%d",
        static_cast<char>('A' + x),
        y + 1
    );
    return true;
}
} // namespace RobotApp

#endif
