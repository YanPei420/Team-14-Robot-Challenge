# ServoControl 库

封装了标准舵机控制逻辑。

## 使用方法

### 1. 初始化
```cpp
#include "ServoControl.h"
#include "config.h"

ServoControl myServo(PIN_SERVO);

void setup() {
    myServo.begin();
}
```

### 2. 控制
```cpp
void loop() {
    myServo.setAngle(90);  // 归中
    delay(1000);
    myServo.setAngle(0);   // 最小值
    delay(1000);
    myServo.setAngle(180); // 最大值
    delay(1000);
}
```

## 功能说明
- `setAngle(int angle)`: 设置舵机目标角度（0-180度）。
