# WiFiHandler 库

用于管理 Giga R1 的 WiFi 连接和基础 UDP 通信。

## 配置
在 `include/arduino_secrets.h` 中设置你的 WiFi 凭据：
```cpp
#define SECRET_SSID "你的WiFi名称"
#define SECRET_PASS "你的密码"
```

在 `include/config.h` 中设置 UDP 端口：
```cpp
const unsigned int PORT_UDP = 2390;
```

## 使用方法

### 1. 初始化
```cpp
#include "wifi_handler.h"

void setup() {
    Serial.begin(115200);
    setupWiFi();
}
```

### 2. 循环处理
```cpp
void loop() {
    handleUDP(); // 持续监听并回复 UDP 消息
}
```

## 功能说明
- `setupWiFi()`: 自动连接 WiFi 并开启指定端口的 UDP 监听。
- `handleUDP()`: 解析收到的 UDP 数据包，打印内容并自动回复 "acknowledged"。
- `printWifiStatus()`: 在串口打印当前的 IP 地址和信号强度。
