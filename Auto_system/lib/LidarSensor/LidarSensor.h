#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include <Arduino.h>

/**
 * @brief TF-Luna 激光雷达传感器驱动类
 * 
 * 通过 Serial1 与 TF-Luna 进行通信，读取距离、信号强度和温度。
 */
class LidarSensor {
public:
    LidarSensor();

    /**
     * @brief 初始化激光雷达串口
     */
    void begin();

    /**
     * @brief 更新传感器数据 (需在 loop 中持续调用)
     */
    void update();

    /**
     * @brief 获取当前探测距离
     * @return int 距离值 (单位: cm)
     */
    int getDistance() const;

    /**
     * @brief 获取信号强度
     * @return int 强度值
     */
    int getAmplitude() const;

    /**
     * @brief 获取传感器温度
     * @return float 温度值 (Celsius)
     */
    float getTemperature() const;

    /**
     * @brief 检查信号是否可靠
     * @return true 信号强度足够
     */
    bool isReliable() const;

private:
    int _distance;
    int _amplitude;
    float _temperature;
};

#endif
