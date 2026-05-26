#if defined(CORE_CM4)

#include "M4_main.h"

#include "M4MotorService.h"

namespace M4Core
{
void setup()
{
    M4MotorService::setup();
}

void loop()
{
    M4MotorService::loop();
}
} // namespace M4Core

#endif
