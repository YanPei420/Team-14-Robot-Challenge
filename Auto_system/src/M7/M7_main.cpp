#if defined(CORE_CM7)

#include "M7_main.h"

#include <Arduino.h>
#include <RPC.h>

#include "../fsm/FSM_main.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
} // namespace

namespace M7Core
{
void setup()
{
    Serial.begin(SERIAL_BAUD);

    const uint32_t serialStartMs = millis();
    while (!Serial && millis() - serialStartMs < 3000)
    {
        ;
    }

    if (!RPC.begin())
    {
        Serial.println("RPC initialization failed; M4 chassis service is offline");
    }

    RobotApp::fsmSetup();
}

void loop()
{
    RobotApp::fsmLoop();
}
} // namespace M7Core

#endif
