#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// 引脚定义 (Pin Definitions)
// =============================================================================

// --- 电机编码器 (Motor Encoders) ---
// Front Left (左前)
constexpr int PIN_ENCODER_FL_A = 22;
constexpr int PIN_ENCODER_FL_B = 23;

// Front Right (右前)
constexpr int PIN_ENCODER_FR_A = 24;
constexpr int PIN_ENCODER_FR_B = 25;

// Rear Left (左后)
constexpr int PIN_ENCODER_RL_A = 52;
constexpr int PIN_ENCODER_RL_B = 53;

// Rear Right (右后)
constexpr int PIN_ENCODER_RR_A = 50;
constexpr int PIN_ENCODER_RR_B = 51;

// --- 舵机与控制 (Servo & Control) ---
constexpr int PIN_SERVO = 9;
constexpr int PIN_KILL_SWITCH = 2;

// --- 传感器 (Sensors) ---
// 红外传感器 (IR Sensor)
constexpr int PIN_IR_SENSOR = A1;

// GP2Y0E03 距离传感器 (Distance Sensor)
constexpr int PIN_DIST_SENSOR_GPIO1 = 26; 
constexpr int PIN_DIST_SENSOR_ANALOG = A0; 

// --- RGB LED (PWM) ---
constexpr int PIN_RGB_R = 3;
constexpr int PIN_RGB_G = 4;
constexpr int PIN_RGB_B = 5;

// =============================================================================
// I2C 设备地址 (I2C Addresses)
// =============================================================================

// Motoron 电机驱动器 (Motoron Motor Controllers)
constexpr uint8_t ADDR_MOTORON_FRONT = 16;
constexpr uint8_t ADDR_MOTORON_REAR = 17;

// GP2Y0E03 距离传感器
constexpr uint8_t ADDR_DIST_SENSOR = 0x40;

// =============================================================================
// 运行参数 (Operational Parameters)
// =============================================================================

// --- 运动速度 (Movement Speeds) ---
constexpr int SPEED_DEFAULT = 400;
constexpr int SPEED_STRAFE = 400;
constexpr int SPEED_TURN = 300;
constexpr int SPEED_MAX = 800;

// --- 传感器阈值 (Sensor Thresholds) ---
constexpr int THRESHOLD_IR = 500;
constexpr int THRESHOLD_LIDAR = 20; // 单位: cm

// --- 通信 (Communication) ---
constexpr unsigned int PORT_UDP = 2390;

// --- 硬件设置 (Hardware Settings) ---
// RGB LED 类型 (true 为共阳极, false 为共阴极)
constexpr bool LED_COMMON_ANODE = false;

#endif // CONFIG_H
