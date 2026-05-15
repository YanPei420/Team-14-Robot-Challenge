#ifndef WIFI_HANDLER_CONFIG_H
#define WIFI_HANDLER_CONFIG_H

#include <Arduino.h>

// ======================================================
// WIFI
// ======================================================

constexpr const char* WIFI_SSID = "PhaseSpaceNetwork_2.4G";
constexpr const char* WIFI_PASSWORD = "8igMacNet";

// constexpr const char* WIFI_SSID = "Xiao Mi 15 Ultra";
// constexpr const char* WIFI_PASSWORD = "00000000";


constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 500;
constexpr uint8_t WIFI_MAX_CONNECT_ATTEMPTS = 3;

// ======================================================
// UDP
// ======================================================

constexpr uint16_t WIFI_UDP_PORT = 4210;
constexpr size_t UDP_BUFFER_SIZE = 255;

// ======================================================
// COMMANDS
// ======================================================

constexpr const char* UDP_STOP_COMMAND = "Stop";

#endif
