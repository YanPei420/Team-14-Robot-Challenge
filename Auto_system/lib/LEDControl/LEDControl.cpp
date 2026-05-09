#include "LEDControl.h"

LEDControl::LEDControl(int pinR, int pinG, int pinB, bool commonAnode)
    : _pinR(pinR), _pinG(pinG), _pinB(pinB), _commonAnode(commonAnode),
      _isBlinking(false), _blinkInterval(500), _lastBlinkTime(0), _blinkState(false) {}

void LEDControl::begin() {
    // 设置所有引脚为输出
    pinMode(_pinR, OUTPUT);
    pinMode(_pinG, OUTPUT);
    pinMode(_pinB, OUTPUT);
    _isBlinking = false;
    // 初始关闭
    off();
}

void LEDControl::setColor(int r, int g, int b) {
    stopBlinking();
    _write(_pinR, r);
    _write(_pinG, g);
    _write(_pinB, b);
}

void LEDControl::update() {
    if (!_isBlinking) return;

    if (millis() - _lastBlinkTime >= _blinkInterval) {
        _lastBlinkTime = millis();
        _blinkState = !_blinkState;
        if (_blinkState) {
            _write(_pinR, 255);
            _write(_pinG, 0);
            _write(_pinB, 0);
        } else {
            _write(_pinR, 0);
            _write(_pinG, 0);
            _write(_pinB, 0);
        }
    }
}

void LEDControl::blinkRed(unsigned long interval) {
    _isBlinking = true;
    _blinkInterval = interval;
}

void LEDControl::stopBlinking() {
    _isBlinking = false;
}

void LEDControl::off()     { setColor(0, 0, 0); }
void LEDControl::red()     { setColor(255, 0, 0); }
void LEDControl::green()   { setColor(0, 255, 0); }
void LEDControl::blue()    { setColor(0, 0, 255); }
void LEDControl::yellow()  { setColor(255, 255, 0); }
void LEDControl::cyan()    { setColor(0, 255, 255); }
void LEDControl::magenta() { setColor(255, 0, 255); }
void LEDControl::white()   { setColor(255, 255, 255); }

void LEDControl::_write(int pin, int value) {
    if (_commonAnode) {
        // 共阳极：255 为关，0 为全亮
        analogWrite(pin, 255 - value);
    } else {
        // 共阴极：255 为全亮，0 为关
        analogWrite(pin, value);
    }
}
