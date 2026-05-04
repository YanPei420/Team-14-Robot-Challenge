#include "wifi_handler.h"
#include "arduino_secrets.h"
#include "config.h"

// WiFi网络信息
char ssid[] = SECRET_SSID;        
char pass[] = SECRET_PASS;    
int status = WL_IDLE_STATUS;

// UDP通信缓冲区
char packetBuffer[256]; 
char ReplyBuffer[] = "acknowledged";       

// UDP对象
WiFiUDP Udp;

// 初始化WiFi连接和UDP监听
void setupWiFi() {
  // 检查WiFi模块
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  // 尝试连接到WiFi网络
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    // 等待10秒连接:
    delay(10000);
  }

  Serial.println("Connected to WiFi");
  printWifiStatus();

  Serial.println("\nStarting UDP listener...");
  Udp.begin(UDP_LOCAL_PORT);
  Serial.print("Listening on port ");
  Serial.println(UDP_LOCAL_PORT);
}

// 处理UDP消息接收和回复
void handleUDP() {
  // 如果有数据可用,读取数据包
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // 将数据包读入缓冲区
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    // 向发送者回复确认消息
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
}

// 打印WiFi连接状态信息
void printWifiStatus() {
  // 打印连接的WiFi网络SSID:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // 打印开发板IP地址:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // 打印信号强度:
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
