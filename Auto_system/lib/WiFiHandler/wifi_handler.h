#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Function to initialize WiFi connection
void setupWiFi();

// Function to handle UDP communication
void handleUDP();

// Print WiFi status to Serial
void printWifiStatus();

#endif // WIFI_HANDLER_H
