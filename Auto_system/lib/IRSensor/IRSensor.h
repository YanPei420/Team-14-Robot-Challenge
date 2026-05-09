#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>

/**
 * @brief 红外传感器驱动类
 * 
 * 用于读取通用模拟红外传感器的数据。
 */
class IRSensor {
public:
    /**
     * @brief 构造函数
     * @param pin 连接传感器的模拟引脚
     */
    IRSensor(int pin);

    /**
     * @brief 初始化传感器引脚
     */
    void begin();

    /**
     * @brief 读取距离原始值
     * @return int 模拟读取的原始数值 (通常为 0-1023 或更高，取决于 ADC 分辨率)
     */
    int readDistance();

private:
    int _pin; // 传感器引脚
};

#endif
