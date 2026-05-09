#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "MotorControl.h"

MotorControl robot;

void setup() {
    Serial.begin(115200);
    // 确保使用 Wire1 并在 MotorControl 初始化前启动
    Wire1.begin(); 
    
    robot.begin();
    
    Serial.println("Robot Ready: Mecanum Full Movement Test");
    Serial.println("Using constants from config.h");
}

void loop() {
    // 1. 前进
    Serial.println("Action: Forward");
    robot.moveForward(SPEED_DEFAULT);
    delay(2000);
    robot.stop();
    delay(500);

    // 2. 向右平移
    Serial.println("Action: Strafe Right");
    robot.moveRight(SPEED_STRAFE);
    delay(2000);
    robot.stop();
    delay(500);

    // 3. 原地顺时针转
    Serial.println("Action: Turn CW");
    robot.turn(90, SPEED_TURN); 
    delay(1000);
    robot.stop();
    delay(500);

    // 4. 原地逆时针转
    Serial.println("Action: Turn CCW");
    robot.turn(-90, SPEED_TURN);
    delay(1000);
    robot.stop();

    delay(3000);
}
