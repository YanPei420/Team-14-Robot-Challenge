#include "LEDControl.h"

LEDControl::LEDControl(int pinR, int pinG, int pinB, bool commonAnode)
    : _pinR(pinR), _pinG(pinG), _pinB(pinB), _commonAnode(commonAnode) {}

void LEDControl::begin() {
    pinMode(_pinR, OUTPUT);
    pinMode(_pinG, OUTPUT);
    pinMode(_pinB, OUTPUT);
    off();
}

void LEDControl::setColor(int r, int g, int b) {
    _write(_pinR, r);
    _write(_pinG, g);
    _write(_pinB, b);
}

void LEDControl::off() {
    setColor(0, 0, 0);
}

void LEDControl::red() {
    setColor(255, 0, 0);
}

void LEDControl::green() {
    setColor(0, 255, 0);
}

void LEDControl::blue() {
    setColor(0, 0, 255);
}

void LEDControl::yellow() {
    setColor(255, 255, 0);
}

void LEDControl::cyan() {
    setColor(0, 255, 255);
}

void LEDControl::magenta() {
    setColor(255, 0, 255);
}

void LEDControl::white() {
    setColor(255, 255, 255);
}

void LEDControl::_write(int pin, int value) {
    if (_commonAnode) {
        analogWrite(pin, 255 - value);
    } else {
        analogWrite(pin, value);
    }
}
