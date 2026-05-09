#ifndef KILL_SWITCH_H
#define KILL_SWITCH_H

#include <Arduino.h>

/**
 * @brief 紧急停止开关 (Kill Switch) 驱动类
 * 
 * 用于监控机器人的物理紧急停止状态。
 */
class KillSwitch {
public:
    /**
     * @brief 构造函数
     * @param pin 连接开关的数字引脚
     */
    KillSwitch(int pin);

    /**
     * @brief 初始化开关引脚
     */
    void begin();

    /**
     * @brief 检查是否已触发紧急停止
     * @return true 处于停止状态 (触发)
     * @return false 处于正常状态
     */
    bool isKilled();

    /**
     * @brief 手动触发紧急停止
     */
    void kill();

    /**
     * @brief 重置紧急停止状态
     */
    void reset();

    /**
     * @brief 打印当前状态到串口
     */
    void printStatus();

private:
    int _pin;      // 引脚号
    bool _killed;  // 当前状态标志
};

#endif
