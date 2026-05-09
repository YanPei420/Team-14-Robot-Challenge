# IRSensor Library

## 文件夹作用
封装了红外避障传感器的读取逻辑。该模块适用于通用的模拟输出红外传感器（如夏普 GP2Y 系列）。

## 主要函数
- `begin()`: 初始化引脚模式。
- `readDistance()`: 读取模拟引脚的原始 ADC 数值（0-4095，基于 Giga 的 12位分辨率）。

## 配置
引脚定义在 `include/config.h` 中的 `PIN_IR_SENSOR`。
