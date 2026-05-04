#include <Arduino.h>
#include "config.h"
#include "arduino_secrets.h"
#include "wifi_handler.h"
#include "KillSwitch.h"

// 全局对象
KillSwitch killSwitch(PIN_KILL_SWITCH);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 10000) {
    ; // 等待串口连接
  }

  Serial.println("\n\n系统初始化...");

  // 首先初始化紧急停止开关（安全优先）
  killSwitch.begin();

  // 初始化 WiFi 和 UDP 监听
  setupWiFi();

  Serial.println("系统就绪！");
}

void loop() {
  // 检查紧急停止开关状态 - 最高优先级
  if (killSwitch.isKilled()) {
    killSwitch.printStatus();
    delay(100);
    return; // 跳过所有其他操作
  }

  // 处理 UDP 数据包
  handleUDP();

  delay(20); 
}