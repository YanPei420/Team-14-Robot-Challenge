#include <Arduino.h>
#include "config.h"

// 重定向 I2C 到 Wire1
#define Wire Wire1
#include <Motoron.h>

// 包含所有模块
#include "MotorControl.h"
#include "IRSensor.h"
#include "DistanceSensor.h"
#include "LidarSensor.h"
#include "KillSwitch.h"
#include "LEDControl.h"
#include "wifi_handler.h"
#include "ServoControl.h"
#include "fsm/StateMachine.h"

// ===== 全局对象实例化 =====
MotorControl motors;
IRSensor irSensor(PIN_IR_SENSOR);
DistanceSensor distSensor(ADDR_DIST_SENSOR, PIN_DIST_SENSOR_GPIO1, PIN_DIST_SENSOR_ANALOG);
LidarSensor lidarSensor;
KillSwitch killSwitch(PIN_KILL_SWITCH);
LEDControl statusLED(PIN_RGB_R, PIN_RGB_G, PIN_RGB_B, LED_COMMON_ANODE);
ServoControl headServo(PIN_SERVO);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Robot System Starting...");

    // 初始化所有硬件模块
    statusLED.begin();
    motors.begin();
    irSensor.begin();
    distSensor.begin();
    lidarSensor.begin();
    killSwitch.begin();
    headServo.begin();

    // 舵机归中
    headServo.setAngle(90);

    // 初始化 WiFi
    setupWiFi();

    // 初始化状态机
    setupFSM();

    Serial.println("System Ready!");
}

void loop() {
    // 运行状态机逻辑 (包含通信、传感器读取和动作执行)
    updateFSM();
    
    // 保持主循环轻量
    delay(10);
}
