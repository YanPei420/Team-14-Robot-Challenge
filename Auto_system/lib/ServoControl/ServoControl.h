#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <Servo.h>

/**
 * @brief 舵机控制类
 * 
 * 封装了 Arduino 官方 Servo 库，提供简化的角度控制接口。
 */
class ServoControl {
public:
    /**
     * @brief 构造函数
     * @param pin 舵机连接的引脚
     */
    ServoControl(int pin);

    /**
     * @brief 初始化并连接舵机
     */
    void begin();

    /**
     * @brief 设置舵机角度
     * @param angle 目标角度 (通常为 0-180)
     */
    void setAngle(int angle);
private:
    Servo _servo; // 舵机实例
    int _pin;     // 连接引脚
};

#endif
