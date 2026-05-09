#include "fsm/StateMachine.h"
#include "config.h"
#include "MotorControl.h"
#include "LidarSensor.h"
#include "LEDControl.h"
#include "WiFiHandler.h"
#include "KillSwitch.h"
#include "DistanceSensor.h"
#include "IRSensor.h"

// 引用在 main.cpp 中定义的全局对象
extern MotorControl motors;
extern LidarSensor lidarSensor;
extern LEDControl statusLED;
extern KillSwitch killSwitch;
extern DistanceSensor distSensor;
extern IRSensor irSensor;

static RobotState _currentState = STATE_STOPPED;
static bool _lastKillButtonState = HIGH;

void setupFSM() {
    pinMode(PIN_TAPPING_BUTTON, INPUT_PULLUP);
    _currentState = STATE_STOPPED;
    Serial.println("FSM Initialized: Starting in STOPPED state.");
}

void updateFSM() {
    // 1. 处理拍打按钮 (最高优先级视觉反馈)
    if (digitalRead(PIN_TAPPING_BUTTON) == LOW) {
        statusLED.setColor(0, 255, 0); // 变绿
        return; // 拍下时挂起状态机逻辑的 LED 控制
    }

    // 2. 状态转换逻辑
    // A. UDP 指令
    String cmd = handleUDP();
    if (cmd == "STOP" || cmd == "Stop") {
        _currentState = STATE_STOPPED;
        Serial.println("UDP STOP received!");
    } else if (cmd == "START" || cmd == "Start") {
        if (_currentState == STATE_STOPPED) {
            _currentState = STATE_RUNNING;
            Serial.println("UDP START received!");
        }
    }

    // B. 物理按钮 Toggle (复用 KillSwitch 的逻辑或直接读取)
    bool currentKillBtn = digitalRead(PIN_KILL_SWITCH);
    if (currentKillBtn == LOW && _lastKillButtonState == HIGH) {
        delay(50); // 防抖
        if (_currentState == STATE_STOPPED) {
            _currentState = STATE_RUNNING;
            Serial.println("Physical START!");
        } else {
            _currentState = STATE_STOPPED;
            Serial.println("Physical STOP!");
        }
    }
    _lastKillButtonState = currentKillBtn;

    // C. 传感器触发转向 (仅在运行时)
    if (_currentState == STATE_RUNNING) {
        lidarSensor.update();
        if (lidarSensor.isReliable() && lidarSensor.getDistance() < 15) {
            _currentState = STATE_AVOIDING;
            Serial.println("Obstacle detected! Switching to AVOIDING.");
        }
    }

    // 3. 执行当前状态动作
    switch (_currentState) {
        case STATE_STOPPED:
            motors.stop();
            statusLED.blinkRed(500);
            break;

        case STATE_RUNNING:
            statusLED.stopBlinking();
            statusLED.red(); // 运行态常亮红灯 (Checklist 1.4)
            motors.moveForward(SPEED_DEFAULT);
            break;

        case STATE_AVOIDING:
            statusLED.red();
            motors.stop();
            delay(500);
            motors.turn(180, SPEED_TURN); // 执行 U-Turn
            delay(2000); // 等待转向完成 (实际应使用编码器)
            _currentState = STATE_RUNNING;
            break;

        case STATE_PLANTING:
            motors.stop();
            // 这里可以添加播种逻辑
            delay(1000);
            _currentState = STATE_STOPPED;
            break;
    }
    
    statusLED.update(); // 处理闪烁
}

RobotState getCurrentState() {
    return _currentState;
}

void setRobotState(RobotState newState) {
    _currentState = newState;
}
