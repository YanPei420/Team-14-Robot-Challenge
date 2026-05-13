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

    Serial.println("Robot ready");
}

void loop()
{
    Robot.forward(100);
    delay(1000);

    Robot.stop_all();
    delay(500);

    Robot.forward(400);
    delay(1000);

    Robot.stop_all();
    delay(500);

    Robot.left(400);
    delay(1000);

    Robot.stop_all();
    delay(500);

    Robot.right(400);
    delay(1000);

    Robot.stop_all();
    delay(500);
};