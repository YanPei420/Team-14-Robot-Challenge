# LidarSensor (TF-Luna)

## 文件夹作用
本文件夹实现了 **TF-Luna** 激光雷达传感器的驱动逻辑。

## 使用说明
该模块通过 Arduino Giga 的 `Serial1` 接口（TX1/RX1）与 TF-Luna 进行串口通信。

### 接口函数
- `begin()`: 初始化 `Serial1` 串口（波特率 115200）。
- `update()`: 解析 9 字节的数据帧，更新内部状态。**必须在 `loop()` 中频繁调用以避免串口缓冲区溢出。**
- `getDistance()`: 返回最近一次读取的距离值（cm）。
- `getAmplitude()`: 返回信号强度（Amp）。
- `getTemperature()`: 返回传感器内部温度。
- `isReliable()`: 检查信号强度是否足以信任当前距离数据（阈值为 100）。

### 硬件连接
- TF-Luna TX -> Arduino RX1 (Pin 19)
- TF-Luna RX -> Arduino TX1 (Pin 18)
- TF-Luna 5V -> Arduino 5V
- TF-Luna GND -> Arduino GND
