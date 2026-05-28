#include <Arduino.h>
#include "MotoronDrive.h"

MotoronDrive Robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    Robot.begin();
}

void loop()
{
    Robot.rotate_left(500);
    delay(1000);
}
