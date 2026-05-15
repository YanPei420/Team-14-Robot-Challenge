#include <Arduino.h>

#include "MotorConfig.h"
#include "MotoronDrive.h"

MotoronDrive Robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
        ;
    }

    Robot.begin();

    Serial.println("FORWARD SPEED TEST");
}

void loop()
{
    // ======================================================
    // SPEED 100
    // ======================================================

    Serial.println("FORWARD 100");

    Robot.forward(100);

    delay(3000);

    Robot.stop_all();

    delay(1000);

    // ======================================================
    // SPEED 500
    // ======================================================

    Serial.println("FORWARD 500");

    Robot.forward(500);

    delay(3000);

    Robot.stop_all();

    delay(2000);
}
