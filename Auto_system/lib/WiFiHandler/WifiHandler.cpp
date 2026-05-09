#include "WifiHandler.h"
#include "arduino_secrets.h"
#include "config.h"

// WiFi 网络信息
char ssid[] = SECRET_SSID;        
char pass[] = SECRET_PASS;    
int status = WL_IDLE_STATUS;

// UDP 通信缓冲区
char packetBuffer[256]; 
char ReplyBuffer[] = "acknowledged";       

// UDP 对象实例
WiFiUDP Udp;

void setupWiFi() {
  // 检查 WiFi 模块是否存在
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("与 WiFi 模块通信失败！");
    while (true);
  }

  // 循环尝试连接 WiFi
  while (status != WL_CONNECTED) {
    Serial.print("正在尝试连接 SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    // 等待 10 秒进行连接:
    delay(10000);
  }

  Serial.println("WiFi 已连接");
  printWifiStatus();

  // 启动 UDP 监听
  Serial.println("\n正在启动 UDP 监听...");
  Udp.begin(PORT_UDP);
  Serial.print("监听端口: ");
  Serial.println(PORT_UDP);
}

String handleUDP() {
  // 检查是否有数据包可用
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("收到数据包，大小: ");
    Serial.println(packetSize);
    Serial.print("来自 IP: ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", 端口: ");
    Serial.println(Udp.remotePort());

    // 将内容读入缓冲区
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0; // 添加字符串结束符
    }
    Serial.println("内容:");
    Serial.println(packetBuffer);

    // 向发送者回复确认消息
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
    
    return String(packetBuffer);
  }
  return "";
}

void printWifiStatus() {
  // 打印 SSID
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // 打印本地 IP 地址
  IPAddress ip = WiFi.localIP();
  Serial.print("IP 地址: ");
  Serial.println(ip);

  // 打印信号强度
  long rssi = WiFi.RSSI();
  Serial.print("信号强度 (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
}
