#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

/**
 * @brief GP2Y0E03 距离传感器驱动类
 * 
 * 支持通过 I2C 和模拟引脚 (Analog) 读取距离数据。
 */
class DistanceSensor {
public:
    /**
     * @brief 构造函数
     * @param addr I2C 设备地址
     * @param gpioPin GPIO1 引脚 (用于使能/唤醒)
     * @param analogPin 模拟输入引脚
     */
    DistanceSensor(uint8_t addr, int gpioPin, int analogPin);

    /**
     * @brief 初始化传感器 (设置引脚模式并唤醒)
     */
    void begin();

    /**
     * @brief 唤醒传感器 (进入活动状态)
     * @return true 唤醒成功
     * @return false 唤醒失败 (I2C 通信异常)
     */
    bool wakeUp();

    /**
     * @brief 通过 I2C 读取距离
     * @return float 距离值 (单位: cm)，若超出范围 (>64cm) 返回值可能不准确
     */
    float readDistanceI2C();

    /**
     * @brief 读取模拟输出电压
     * @return float 电压值 (单位: V)
     */
    float readVoltage();

    /**
     * @brief 读取原始模拟数值
     * @return int 原始 ADC 数值
     */
    int readRawAnalog();

private:
    uint8_t _addr;    // I2C 地址
    int _gpioPin;     // 使能引脚
    int _analogPin;   // 模拟读取引脚
};

#endif
