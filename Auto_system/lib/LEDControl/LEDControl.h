#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>

/**
 * @brief RGB LED 控制类
 * 
 * 提供简便的方法来控制 RGB LED 的颜色，支持共阴极和共阳极 LED。
 */
class LEDControl {
public:
    /**
     * @brief 构造函数
     * @param pinR 红色引脚
     * @param pinG 绿色引脚
     * @param pinB 蓝色引脚
     * @param commonAnode 是否为共阳极 (true: 共阳, false: 共阴)
     */
    LEDControl(int pinR, int pinG, int pinB, bool commonAnode = false);

    /**
     * @brief 初始化 LED 引脚
     */
    void begin();

    /**
     * @brief 设置自定义 RGB 颜色
     * @param r 红色亮度 (0-255)
     * @param g 绿色亮度 (0-255)
     * @param b 蓝色亮度 (0-255)
     */
    void setColor(int r, int g, int b);

    /**
     * @brief 关闭所有 LED
     */
    void off();

    /**
     * @brief 设置为红色
     */
    void red();

    /**
     * @brief 设置为绿色
     */
    void green();

    /**
     * @brief 设置为蓝色
     */
    void blue();

    /**
     * @brief 设置为黄色
     */
    void yellow();

    /**
     * @brief 设置为青色
     */
    void cyan();

    /**
     * @brief 设置为品红色
     */
    void magenta();

    /**
     * @brief 更新 LED 状态 (用于处理闪烁等异步逻辑)
     */
    void update();

    /**
     * @brief 开始红灯闪烁
     * @param interval 闪烁间隔 (ms)
     */
    void blinkRed(unsigned long interval);

    /**
     * @brief 停止闪烁
     */
    void stopBlinking();

private:
    int _pinR, _pinG, _pinB;
    bool _commonAnode;
    
    // 闪烁逻辑相关
    bool _isBlinking;
    unsigned long _blinkInterval;
    unsigned long _lastBlinkTime;
    bool _blinkState;

    /**
     * @brief 写入引脚亮度的私有方法 (处理共阳/共阴逻辑转换)
     */
    void _write(int pin, int value);
};

#endif
