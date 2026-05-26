#pragma once

#if defined(CORE_CM7)

#include <Arduino.h>
#include <MiniMessenger.h>

namespace RobotApp
{
struct RemoteEvents
{
    bool start = false;
    bool exitClearance = false;
    bool exitDoorDetected = false;
    bool exitDoorOpened = false;
    bool mainArenaReached = false;
    bool emergencyReturn = false;
    bool entryAirlockReached = false;
    bool entryDoorOpened = false;
    bool baseReached = false;
    bool stranded = false;
    bool revive = false;
    bool rfid = false;
    bool rfidFertile = false;
    char rfidCoordinate[4] = "";
};

class RemoteMessageParser
{
public:
    void copyPayload(const uint8_t* payload, size_t length);
    bool messageFieldEquals(const char* key, const char* value) const;
    void parseInto(RemoteEvents& events) const;
    const char* message() const;

private:
    char lastMessage_[MiniMessenger::kMaxPayloadSize + 1] = "";

    bool messageContains(const char* token) const;
    void copyCoordinateFromMessage(
        char* destination,
        size_t destinationSize
    ) const;
};
} // namespace RobotApp

#endif
