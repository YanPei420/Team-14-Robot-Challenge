#include "MiniMessenger.h"

#include <cstdio>
#include <cstring>

MiniMessenger::MiniMessenger()
    : _mqttClient(_wifiClient),
      _callback(nullptr),
      _brokerPort(1883),
      _hasBegun(false),
      _subscriptionsReady(false),
      _lastWifiAttemptMs(0),
      _lastMqttAttemptMs(0) {
  _ssid[0] = '\0';
  _password[0] = '\0';
  _brokerHost[0] = '\0';
  _groupId[0] = '\0';
  _boardId[0] = '\0';
  _clientId[0] = '\0';
  _statusTopic[0] = '\0';
  _boardSubscription[0] = '\0';
  _groupSubscription[0] = '\0';
}

bool MiniMessenger::begin(const char* ssid,
                          const char* password,
                          const char* brokerHost,
                          uint16_t brokerPort,
                          const char* groupId,
                          const char* boardId) {
  copyString(_ssid, sizeof(_ssid), ssid);
  copyString(_password, sizeof(_password), password);
  copyString(_brokerHost, sizeof(_brokerHost), brokerHost);
  copyString(_groupId, sizeof(_groupId), groupId);
  copyString(_boardId, sizeof(_boardId), boardId);
  _brokerPort = brokerPort;

  buildClientId();
  buildTopics();

  _hasBegun = true;
  _subscriptionsReady = false;
  _lastWifiAttemptMs = 0;
  _lastMqttAttemptMs = 0;

  ensureWifiConnected();
  ensureMqttConnected();
  return isConnected();
}

void MiniMessenger::loop() {
  if (!_hasBegun) {
    return;
  }

  ensureWifiConnected();
  ensureMqttConnected();

  if (_mqttClient.connected()) {
    pollMessages();
  }
}

bool MiniMessenger::sendToBoard(const char* targetBoardId,
                                const uint8_t* data,
                                size_t length) {
  if (!targetBoardId || !data || length == 0) {
    return false;
  }

  char topic[96];
  snprintf(topic,
           sizeof(topic),
           "lab/g/%s/from/%s/to/%s",
           _groupId,
           _boardId,
           targetBoardId);
  return publishRaw(topic, data, length, false, 0);
}

bool MiniMessenger::sendToBoard(const char* targetBoardId, const char* text) {
  if (!text) {
    return false;
  }

  return sendToBoard(targetBoardId,
                     reinterpret_cast<const uint8_t*>(text),
                     strlen(text));
}

bool MiniMessenger::sendToBoard(uint8_t targetBoardId,
                                const uint8_t* data,
                                size_t length) {
  char target[8];
  snprintf(target, sizeof(target), "%u", targetBoardId);
  return sendToBoard(target, data, length);
}

bool MiniMessenger::sendToBoard(uint8_t targetBoardId, const char* text) {
  char target[8];
  snprintf(target, sizeof(target), "%u", targetBoardId);
  return sendToBoard(target, text);
}

bool MiniMessenger::sendToGroup(const uint8_t* data, size_t length) {
  if (!data || length == 0) {
    return false;
  }

  char topic[96];
  snprintf(
      topic, sizeof(topic), "lab/g/%s/from/%s/to/all", _groupId, _boardId);
  return publishRaw(topic, data, length, false, 0);
}

bool MiniMessenger::sendToGroup(const char* text) {
  if (!text) {
    return false;
  }

  return sendToGroup(reinterpret_cast<const uint8_t*>(text), strlen(text));
}

void MiniMessenger::onMessage(MessageCallback callback) {
  _callback = callback;
}

bool MiniMessenger::isConnected() {
  return WiFi.status() == WL_CONNECTED && _mqttClient.connected();
}

const char* MiniMessenger::clientId() const { return _clientId; }
const char* MiniMessenger::groupId() const { return _groupId; }
const char* MiniMessenger::boardId() const { return _boardId; }

void MiniMessenger::copyString(char* destination,
                               size_t destinationSize,
                               const char* source) {
  if (!destination || destinationSize == 0) {
    return;
  }

  if (!source) {
    destination[0] = '\0';
    return;
  }

  strncpy(destination, source, destinationSize - 1);
  destination[destinationSize - 1] = '\0';
}

void MiniMessenger::buildClientId() {
  snprintf(_clientId,
           sizeof(_clientId),
           "g%02d-b%02d",
           atoi(_groupId),
           atoi(_boardId));
}

void MiniMessenger::buildTopics() {
  snprintf(_statusTopic,
           sizeof(_statusTopic),
           "lab/g/%s/board/%s/status",
           _groupId,
           _boardId);

  snprintf(_boardSubscription,
           sizeof(_boardSubscription),
           "lab/g/%s/from/+/to/%s",
           _groupId,
           _boardId);

  snprintf(_groupSubscription,
           sizeof(_groupSubscription),
           "lab/g/%s/from/+/to/all",
           _groupId);
}

void MiniMessenger::ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const unsigned long now = millis();
  if (now - _lastWifiAttemptMs < kWifiRetryIntervalMs) {
    return;
  }

  _lastWifiAttemptMs = now;
  _subscriptionsReady = false;
  WiFi.disconnect();
  WiFi.begin(_ssid, _password);
}

void MiniMessenger::ensureMqttConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (_mqttClient.connected()) {
    if (!_subscriptionsReady) {
      subscribeTopics();
    }
    return;
  }

  _subscriptionsReady = false;
  const unsigned long now = millis();
  if (now - _lastMqttAttemptMs < kMqttRetryIntervalMs) {
    return;
  }

  _lastMqttAttemptMs = now;
  connectMqtt();
}

bool MiniMessenger::connectMqtt() {
  _mqttClient.stop();
  _mqttClient.setId(_clientId);

  const char* willPayload = "offline";
  _mqttClient.beginWill(
      _statusTopic, strlen(willPayload), true, 0);
  _mqttClient.print(willPayload);
  _mqttClient.endWill();

  if (!_mqttClient.connect(_brokerHost, _brokerPort)) {
    return false;
  }

  _subscriptionsReady = false;
  if (!subscribeTopics()) {
    _mqttClient.stop();
    return false;
  }
  publishStatus("online");
  return true;
}

bool MiniMessenger::publishStatus(const char* status) {
  if (!status) {
    return false;
  }

  return publishRaw(
      _statusTopic,
      reinterpret_cast<const uint8_t*>(status),
      strlen(status),
      true,
      1);
}

bool MiniMessenger::publishRaw(const char* topic,
                               const uint8_t* data,
                               size_t length,
                               bool retain,
                               uint8_t qos) {
  if (!_mqttClient.connected() || !topic || !data || length == 0) {
    return false;
  }

  if (!_mqttClient.beginMessage(
          topic, static_cast<unsigned long>(length), retain, qos, false)) {
    return false;
  }

  const size_t written = _mqttClient.write(data, length);
  if (written != length) {
    _mqttClient.stop();
    return false;
  }

  return _mqttClient.endMessage() == 1;
}

void MiniMessenger::pollMessages() {
  const int messageSize = _mqttClient.parseMessage();
  if (messageSize <= 0) {
    return;
  }

  MessageMetadata metadata = {};
  const String topic = _mqttClient.messageTopic();
  if (!parseMessageTopic(topic, metadata)) {
    while (_mqttClient.available()) {
      _mqttClient.read();
    }
    return;
  }

  if (messageSize > static_cast<int>(kMaxPayloadSize)) {
    while (_mqttClient.available()) {
      _mqttClient.read();
    }
    return;
  }

  uint8_t payload[kMaxPayloadSize];
  size_t offset = 0;
  while (_mqttClient.available() && offset < sizeof(payload)) {
    const int value = _mqttClient.read();
    if (value < 0) {
      break;
    }
    payload[offset++] = static_cast<uint8_t>(value);
  }

  if (_callback && offset > 0) {
    _callback(metadata, payload, offset);
  }
}

bool MiniMessenger::parseMessageTopic(const String& topic,
                                      MessageMetadata& metadata) {
  char groupId[16];
  char fromBoardId[16];
  char target[16];

  const int matched = sscanf(topic.c_str(),
                             "lab/g/%15[^/]/from/%15[^/]/to/%15[^/]",
                             groupId,
                             fromBoardId,
                             target);

  if (matched == 3) {
    copyString(metadata.groupId, sizeof(metadata.groupId), groupId);
    copyString(metadata.fromBoardId, sizeof(metadata.fromBoardId), fromBoardId);
    copyString(metadata.target, sizeof(metadata.target), target);
    metadata.isGroupMessage = strcmp(target, "all") == 0;
    return true;
  }

  return false;
}

bool MiniMessenger::subscribeTopics() {
  if (_subscriptionsReady) {
    return true;
  }

  if (!_mqttClient.subscribe(_boardSubscription)) {
    return false;
  }

  if (!_mqttClient.subscribe(_groupSubscription)) {
    return false;
  }

  _subscriptionsReady = true;
  return true;
}
