#include <Arduino.h>
#include <unity.h>
#include "wifi_handler.h"
#include "arduino_secrets.h"

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_wifi_connection_status(void) {
  setupWiFi();
  TEST_ASSERT_EQUAL(WL_CONNECTED, WiFi.status());
}

void test_wifi_local_ip(void) {
  IPAddress ip = WiFi.localIP();
  // Check that the IP is not 0.0.0.0 and not INADDR_NONE (255.255.255.255)
  TEST_ASSERT_TRUE((uint32_t)ip != 0x00000000);
  TEST_ASSERT_TRUE((uint32_t)ip != 0xFFFFFFFF);
  Serial.print("Local IP: ");
  Serial.println(ip);
}

void setup() {
  // Wait for Serial to be ready
  Serial.begin(115200);
  while (!Serial && millis() < 5000);

  UNITY_BEGIN();
  RUN_TEST(test_wifi_connection_status);
  RUN_TEST(test_wifi_local_ip);
  UNITY_END();
}

void loop() {
  // Functional test: allow UDP packets to be handled after unit tests finish
  handleUDP();
  delay(100);
}
