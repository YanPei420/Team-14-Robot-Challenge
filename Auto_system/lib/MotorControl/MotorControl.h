#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// 将默认的 I2C 总线重定向到 Wire1 (Arduino Giga 的特定总线)
#define Wire Wire1
#include <Motoron.h>
#include "config.h"

/**
 * @brief 电机控制类
 * 
 * 封装了麦克纳姆轮底盘的运动控制逻辑，支持四轮独立驱动和编码器反馈。
 * 使用两块 Motoron I2C 电机驱动板进行控制。
 */
class MotorControl {
public:
    /**
     * @brief 构造函数
     */
    MotorControl();

    /**
     * @brief 初始化电机驱动器和编码器引脚
     */
    void begin();

    /**
     * @brief 停止所有电机
     */
    void stop();

    /**
     * @brief 前进
     * @param speed 速度 (0-800)
     */
    void moveForward(int speed);

    /**
     * @brief 后退
     * @param speed 速度 (0-800)
     */
    void moveBackward(int speed);

    /**
     * @brief 向左平移
     * @param speed 速度 (0-800)
     */
    void moveLeft(int speed);

    /**
     * @brief 向右平移
     * @param speed 速度 (0-800)
     */
    void moveRight(int speed);

    /**
     * @brief 原地旋转
     * @param angle 旋转角度 (正数为顺时针，负数为逆时针) - 目前为占位，需配合编码器
     * @param speed 速度
     */
    void turn(int angle, int speed);

    /**
     * @brief 直接设置四个电机的速度
     * @param fl 左前
     * @param fr 右前
     * @param rl 左后
     * @param rr 右后
     */
    void setSpeeds(int fl, int fr, int rl, int rr);

    /**
     * @brief 重置编码器计数值
     */
    void resetEncoders();

    /**
     * @brief 获取编码器数值
     */
    long getCountFL() const;
    long getCountFR() const;
    long getCountRL() const;
    long getCountRR() const;

private:
    MotoronI2C _mc1; // 控制前轮 (Address 16)
    MotoronI2C _mc2; // 控制后轮 (Address 17)

    void _initMotoron(MotoronI2C &mc);
};

#endif
