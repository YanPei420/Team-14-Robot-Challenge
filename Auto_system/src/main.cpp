#include <Arduino.h>

#if defined(CORE_CM4)
#include "M4/M4_main.h"
#elif defined(CORE_CM7)
#include "M7/M7_main.h"
#endif

void setup()
{
#if defined(CORE_CM4)
    M4Core::setup();
#elif defined(CORE_CM7)
    M7Core::setup();
#endif
}

void loop()
{
#if defined(CORE_CM4)
    M4Core::loop();
#elif defined(CORE_CM7)
    M7Core::loop();
#endif
}
