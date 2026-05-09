#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "MotorControl.h"

MotorControl robot;

/**
 * 方法一：原地掉头 (In-place Point Turn)
 * 机器人绕中心旋转 180 度。适用于狭窄空间。
 */
void uTurnInPlace(int speed) {
    Serial.println(">>> Method 1: In-place U-Turn (180 deg)");
    // 假设在特定速度下旋转 180 度需要的时间 (需根据实际摩擦力调整)
    // 更好的做法是判断编码器数值：while(abs(robot.getCountFL()) < TARGET_TICKS_180)
    robot.turn(180, speed);
    delay(2000); // 调整此延时直到正好转够 180 度
    robot.stop();
}

/**
 * 方法二：弧线掉头 (Arc Turn / Differential Turn)
 * 类似汽车掉头，通过左右轮速差实现平滑的 U 型路径。
 * 这种方法更平稳，不会因为原地打滑丢失位置信息。
 */
void uTurnArc(int outerSpeed, int innerSpeed) {
    Serial.println(">>> Method 2: Smooth Arc U-Turn");
    // 左转 U 型弯：右轮（外侧）快，左轮（内侧）慢
    // 参数顺序：FL, FR, RL, RR
    // 注意：FR 和 RR 在 Motoron 中需要负值来表示前进
    robot.setSpeeds(innerSpeed, -outerSpeed, innerSpeed, -outerSpeed);
    
    delay(4000); // 调整此延时直到完成 U 型弧线
    robot.stop();
}

void setup() {
    Serial.begin(115200);
    Wire1.begin();
    
    robot.begin();
    
    Serial.println("Mecanum Robot U-Turn Test");
    Serial.println("1. Point Turn (Rotating)");
    Serial.println("2. Arc Turn (Curved)");
    delay(2000);
}

void loop() {
    // 首先向前走一段
    Serial.println("Moving Forward...");
    robot.moveForward(SPEED_DEFAULT);
    delay(1000);

    // 执行方法一
    uTurnInPlace(SPEED_TURN);
    delay(2000);

    // 再向前走一段返回
    Serial.println("Moving Forward...");
    robot.moveForward(SPEED_DEFAULT);
    delay(1000);
    robot.stop();
    delay(2000);

    // --- 分隔 ---

    // 再次向前走
    Serial.println("Moving Forward...");
    robot.moveForward(SPEED_DEFAULT);
    delay(1000);

    // 执行方法二 (假设外侧 500，内侧 100)
    uTurnArc(500, 100);
    delay(2000);

    robot.stop();
    Serial.println("Test Cycle Complete. Waiting 5s...");
    delay(5000);
}
