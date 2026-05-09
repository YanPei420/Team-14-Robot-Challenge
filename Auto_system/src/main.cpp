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

    // 初始化所有模块
    statusLED.begin();
    statusLED.blue(); // 初始状态：蓝色（启动中）

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

    statusLED.green(); // 就绪状态：绿色
    Serial.println("System Ready!");
}

void loop() {
    // 1. 处理网络通信
    handleUDP();

    // 2. 安全检查：紧急停止
    if (killSwitch.isKilled()) {
        motors.stop();
        statusLED.red();
        Serial.println("CRITICAL: Kill switch triggered!");
        while (killSwitch.isKilled()) {
            delay(100);
        }
        statusLED.green();
    }

    // 3. 传感器数据读取
    lidarSensor.update(); // 持续从串口读取最新数据

    int irVal = irSensor.readDistance();
    float distI2C = distSensor.readDistanceI2C();
    int lidarDist = lidarSensor.getDistance();

    // 4. 避障逻辑
    bool obstacleDetected = false;

    // 检查红外传感器
    if (irVal > THRESHOLD_IR) {
        obstacleDetected = true;
        Serial.println("Obstacle: IR Triggered");
    }

    // 检查 I2C 距离传感器
    if (distI2C < THRESHOLD_LIDAR) { 
        obstacleDetected = true;
        Serial.print("Obstacle: I2C Dist (");
        Serial.print(distI2C);
        Serial.println(" cm)");
    }

    // 检查激光雷达 (仅在信号可靠时)
    if (lidarSensor.isReliable() && lidarDist > 0 && lidarDist < THRESHOLD_LIDAR) {
        obstacleDetected = true;
        Serial.print("Obstacle: Lidar Dist (");
        Serial.print(lidarDist);
        Serial.println(" cm)");
    }

    // 5. 运动执行
    if (obstacleDetected) {
        motors.stop();
        statusLED.yellow();
    } else {
        motors.moveForward(SPEED_DEFAULT);
        statusLED.green();
    }

    delay(20); 
}
