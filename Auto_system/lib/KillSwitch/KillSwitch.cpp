#include "KillSwitch.h"

KillSwitch::KillSwitch(int pin) : _pin(pin), _killed(false) {
}

void KillSwitch::begin() {
  // 使用内置上拉电阻，开关另一端应接地
  pinMode(_pin, INPUT_PULLUP);
  Serial.println("KillSwitch 初始化于引脚 " + String(_pin));
}

bool KillSwitch::isKilled() {
  // 读取引脚状态 - LOW 表示触发（按钮按下/接地）
  if (digitalRead(_pin) == LOW) {
    _killed = true;
  }
  return _killed;
}

void KillSwitch::kill() {
  _killed = true;
  Serial.println("紧急停止已激活 (KILL SWITCH ACTIVATED)!");
}

void KillSwitch::reset() {
  _killed = false;
  Serial.println("紧急停止状态已重置。");
}

void KillSwitch::printStatus() {
  Serial.print("KillSwitch 状态: ");
  Serial.println(_killed ? "已停止 (KILLED)" : "正常运行 (ACTIVE)");
}
