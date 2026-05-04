#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// 引脚定义
// =============================================================================

// 舵机控制引脚
const int PIN_SERVO = 9;

// Kill Switch 引脚 (紧急停止)
const int PIN_KILL_SWITCH = 2;

// 红外传感器引脚
const int PIN_IR_SENSOR = A0;

// Motoron 电机驱动器 (I2C)
// SDA = 20, SCL = 21 (Giga R1 的默认 I2C 引脚)

// =============================================================================
// 全局参数
// =============================================================================

// WiFi UDP 端口
const unsigned int UDP_LOCAL_PORT = 2390;

// 默认电机速度
const int MOTOR_SPEED_DEFAULT = 100;

// 障碍物检测阈值
const int OBSTACLE_THRESHOLD_IR = 500;
const int OBSTACLE_THRESHOLD_LIDAR = 20; // 厘米

#endif
