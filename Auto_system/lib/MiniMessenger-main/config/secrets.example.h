#pragma once

// Copy this file to secrets.h (under you project directory) and update the values for the lab network.
// Group ID is used to separate the MQTT topics for different groups.
// Make sure you use your team number as the group ID (e.g., "1", "2", etc.) to avoid conflicts with other teams.
// Do not change the BROKER_HOST unless you have a different MQTT broker setup. 
// The default is set to a local broker on the lab network.

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define BROKER_HOST "192.168.0.74"
#define BROKER_PORT 1883
#define GROUP_ID "1"

