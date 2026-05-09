#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>

/**
 * @brief 机器人运行状态枚举
 */
enum RobotState {
    STATE_STOPPED,    // 停止/急停状态 (红灯闪烁)
    STATE_RUNNING,    // 正常巡线/探索状态 (红灯常亮)
    STATE_AVOIDING,   // 避障转向状态 (红灯常亮)
    STATE_PLANTING    // 播种动作演示 (动作完后回停止)
};

/**
 * @brief 状态机初始化
 */
void setupFSM();

/**
 * @brief 状态机循环逻辑
 */
void updateFSM();

/**
 * @brief 获取当前状态
 */
RobotState getCurrentState();

/**
 * @brief 手动设置状态
 */
void setRobotState(RobotState newState);

#endif
