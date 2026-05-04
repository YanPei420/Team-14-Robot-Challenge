#include "KillSwitch.h"

KillSwitch::KillSwitch(int pin) : _pin(pin), _killed(false) {
}

void KillSwitch::begin() {
  pinMode(_pin, INPUT_PULLUP);
  Serial.println("KillSwitch initialized on pin " + String(_pin));
}

bool KillSwitch::isKilled() {
  // 读取引脚状态 - LOW 表示触发（按钮按下）
  if (digitalRead(_pin) == LOW) {
    _killed = true;
  }
  return _killed;
}

void KillSwitch::kill() {
  _killed = true;
  Serial.println("KILL SWITCH ACTIVATED!");
}

void KillSwitch::reset() {
  _killed = false;
  Serial.println("KillSwitch reset.");
}

void KillSwitch::printStatus() {
  Serial.print("KillSwitch Status: ");
  Serial.println(_killed ? "KILLED" : "ACTIVE");
}
