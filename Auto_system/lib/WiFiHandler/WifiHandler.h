#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

/**
 * @brief 初始化 WiFi 连接并启动 UDP 监听
 */
void setupWiFi();

/**
 * @brief 处理 UDP 消息的接收与回复
 * @return String 接收到的消息内容，若无消息则返回空字符串
 */
String handleUDP();

/**
 * @brief 将当前 WiFi 连接状态打印到串口
 */
void printWifiStatus();

#endif // WIFI_HANDLER_H
