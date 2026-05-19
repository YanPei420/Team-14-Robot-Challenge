#pragma once

#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <WiFi.h>

struct MessageMetadata {
  char groupId[16];
  char fromBoardId[16];
  char target[16];
  bool isGroupMessage;
};

using MessageCallback = void (*)(const MessageMetadata&, const uint8_t*, size_t);

class MiniMessenger {
public:
  static constexpr size_t kMaxPayloadSize = 512;

  MiniMessenger();

  bool begin(const char* ssid,
             const char* password,
             const char* brokerHost,
             uint16_t brokerPort,
             const char* groupId,
             const char* boardId);

  void loop();
  bool sendToBoard(const char* targetBoardId, const uint8_t* data, size_t length);
  bool sendToBoard(const char* targetBoardId, const char* text);
  bool sendToBoard(uint8_t targetBoardId, const uint8_t* data, size_t length);
  bool sendToBoard(uint8_t targetBoardId, const char* text);
  bool sendToGroup(const uint8_t* data, size_t length);
  bool sendToGroup(const char* text);
  void onMessage(MessageCallback callback);
  bool isConnected();

  const char* clientId() const;
  const char* groupId() const;
  const char* boardId() const;

private:
  static constexpr unsigned long kWifiRetryIntervalMs = 5000;
  static constexpr unsigned long kMqttRetryIntervalMs = 3000;

  WiFiClient _wifiClient;
  MqttClient _mqttClient;
  MessageCallback _callback;

  char _ssid[64];
  char _password[64];
  char _brokerHost[64];
  uint16_t _brokerPort;
  char _groupId[16];
  char _boardId[16];
  char _clientId[24];
  char _statusTopic[96];
  char _boardSubscription[96];
  char _groupSubscription[96];
  bool _hasBegun;
  bool _subscriptionsReady;
  unsigned long _lastWifiAttemptMs;
  unsigned long _lastMqttAttemptMs;

  void copyString(char* destination, size_t destinationSize, const char* source);
  void buildClientId();
  void buildTopics();
  void ensureWifiConnected();
  void ensureMqttConnected();
  bool connectMqtt();
  bool publishStatus(const char* status);
  bool publishRaw(const char* topic,
                  const uint8_t* data,
                  size_t length,
                  bool retain,
                  uint8_t qos);
  void pollMessages();
  bool parseMessageTopic(const String& topic, MessageMetadata& metadata);
  bool subscribeTopics();
};
