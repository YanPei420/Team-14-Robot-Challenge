#include "MotorControl.h"

// ===== 编码器计数值 (全局变量以便中断访问) =====
volatile long _countFL = 0;
volatile long _countFR = 0;
volatile long _countRL = 0;
volatile long _countRR = 0;

// ===== 中断服务程序 (ISRs) =====
void _ISR_FL() {
    if (digitalRead(PIN_ENCODER_FL_B)) _countFL++;
    else _countFL--;
}
void _ISR_FR() {
    if (digitalRead(PIN_ENCODER_FR_B)) _countFR++;
    else _countFR--;
}
void _ISR_RL() {
    if (digitalRead(PIN_ENCODER_RL_B)) _countRL++;
    else _countRL--;
}
void _ISR_RR() {
    if (digitalRead(PIN_ENCODER_RR_B)) _countRR++;
    else _countRR--;
}

MotorControl::MotorControl() 
    : _mc1(ADDR_MOTORON_FRONT), _mc2(ADDR_MOTORON_REAR) {}

void MotorControl::begin() {
    // 初始化 I2C (如果尚未初始化)
    // 注意：Wire 已经在 main.cpp 或全局重定向为 Wire1
    
    _initMotoron(_mc1);
    _initMotoron(_mc2);

    // 配置编码器引脚
    pinMode(PIN_ENCODER_FL_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_FL_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_FR_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_FR_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_RL_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_RL_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_RR_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_RR_B, INPUT_PULLUP);

    // 绑定中断
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_FL_A), _ISR_FL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_FR_A), _ISR_FR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RL_A), _ISR_RL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RR_A), _ISR_RR, CHANGE);
}

void MotorControl::_initMotoron(MotoronI2C &mc) {
    mc.reinitialize();
    mc.disableCrc();
    mc.clearResetFlag();
}

void MotorControl::stop() {
    setSpeeds(0, 0, 0, 0);
}

void MotorControl::moveForward(int speed) {
    // Forward: FL+, FR+, RL+, RR+
    // FR and RR are inverted (Forward is -speed)
    setSpeeds(speed, -speed, speed, -speed);
}

void MotorControl::moveBackward(int speed) {
    // Backward: FL-, FR-, RL-, RR-
    setSpeeds(-speed, speed, -speed, speed);
}

void MotorControl::moveLeft(int speed) {
    // Left Strafe: FL-, FR+, RL+, RR-
    setSpeeds(-speed, -speed, speed, speed);
}

void MotorControl::moveRight(int speed) {
    // Right Strafe: FL+, FR-, RL-, RR+
    setSpeeds(speed, speed, -speed, -speed);
}

void MotorControl::turn(int angle, int speed) {
    // Positive angle: Clockwise (FL+, FR-, RL+, RR-)
    // Negative angle: Counter-Clockwise (FL-, FR+, RL-, RR+)
    if (angle > 0) {
        setSpeeds(speed, speed, speed, speed);
    } else {
        setSpeeds(-speed, -speed, -speed, -speed);
    }
}


void MotorControl::setSpeeds(int fl, int fr, int rl, int rr) {
    _mc1.setSpeed(1, fl);
    _mc1.setSpeed(3, fr);
    _mc2.setSpeed(1, rl);
    _mc2.setSpeed(3, rr);
}

void MotorControl::resetEncoders() {
    _countFL = _countFR = _countRL = _countRR = 0;
}

long MotorControl::getCountFL() const { return _countFL; }
long MotorControl::getCountFR() const { return _countFR; }
long MotorControl::getCountRL() const { return _countRL; }
long MotorControl::getCountRR() const { return _countRR; }
