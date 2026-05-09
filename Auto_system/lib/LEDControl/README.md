# LEDControl Library

## 文件夹作用
控制 RGB LED 的颜色显示，用于表示机器人的系统状态（如：运行中-绿色，避障-黄色，急停-红色）。

## 主要函数
- `begin()`: 初始化引脚模式。
- `setColor(r, g, b)`: 设置自定义颜色 (0-255)。
- `red()`, `green()`, `blue()`, `white()`, `off()`: 预设颜色快捷函数。

## 硬件支持
支持**共阳极**和**共阴极** RGB LED。可通过构造函数或 `config.h` 中的 `LED_COMMON_ANODE` 进行配置。
