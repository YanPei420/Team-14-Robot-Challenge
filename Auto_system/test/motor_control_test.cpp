#include <Arduino.h>
#include <MotorControl.h>

MotorControl robot;

void setup() {
    Serial.begin(115200);
    while (!Serial); 
    
    Wire1.begin();
    robot.begin();
    
    Serial.println("Robot Motor Test (ABCD Encapsulation)");
    Serial.println("------------------------------------");
    delay(2000);
}

void loop() {
    // 1. Move Forward
    Serial.println("Moving Forward...");
    robot.moveForward(400);
    delay(2000);
    robot.stop();
    delay(1000);

    // 2. Strafe Right
    Serial.println("Strafing Right...");
    robot.moveRight(400);
    delay(2000);
    robot.stop();
    delay(1000);

    // 3. Turn 90 Degrees Right
    Serial.println("Turning 90 Degrees Right...");
    robot.turn(90, 400); // 90 degrees
    delay(1000);

    // 4. Turn 90 Degrees Left
    Serial.println("Turning 90 Degrees Left...");
    robot.turn(-90, 400); // -90 degrees
    delay(1000);

    // 5. Individual Motor Test (A, B, C, D)
    Serial.println("Individual Motor Test: Motor A only");
    robot.setSpeeds(400, 0, 0, 0); // Only FL
    delay(1000);
    robot.stop();
    
    Serial.println("Test Cycle Complete. Waiting 5s...");
    delay(5000);
}
