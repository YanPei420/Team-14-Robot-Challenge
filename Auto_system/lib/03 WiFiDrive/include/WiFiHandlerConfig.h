#ifndef WIFI_HANDLER_CONFIG_H
#define WIFI_HANDLER_CONFIG_H

#include <Arduino.h>

// ======================================================
// WIFI
// ======================================================

constexpr const char* WIFI_SSID = "PhaseSpaceNetwork_2.4G";
constexpr const char* WIFI_PASSWORD = "8igMacNet";

// ======================================================
// MQTT BROKER
// ======================================================

constexpr const char* BROKER_HOST = "192.168.0.74";
constexpr uint16_t BROKER_PORT = 1883;

// ======================================================
// ROBOT IDENTITY
// ======================================================

constexpr const char* GROUP_ID = "14";
constexpr const char* BOARD_ID = "Robot14";
constexpr const char* SERVER_BOARD_ID = "server";

// ======================================================
// TIMING
// ======================================================

constexpr uint32_t WIFI_REGISTER_INTERVAL_MS = 10000;
constexpr uint32_t WIFI_HEARTBEAT_TIMEOUT_MS = 1000;

#endif
