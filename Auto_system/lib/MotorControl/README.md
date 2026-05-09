# MotorControl Library

## 文件夹作用
`MotorControl` 文件夹封装了基于 **Pololu Motoron** 电机驱动器的麦克纳姆轮（Mecanum Wheel）底盘控制逻辑。它负责：
1. **硬件抽象**：管理两块 Motoron I2C 电机驱动板（mc1 控制前轮，mc2 控制后轮）。
2. **运动学算法**：实现麦克纳姆轮的前进、后退、左右平移以及原地旋转。
3. **闭环反馈**：自动配置四个轮子的编码器引脚（使用硬件中断），并实时维护计数值。
4. **统一配置**：从全局 `config.h` 中读取引脚、I2C 地址及默认速度。

---

## 主要函数使用方法

### 1. 初始化
在 `setup()` 中调用，用于初始化 I2C 通信、电机驱动器状态以及编码器中断。
```cpp
motors.begin();
```

### 2. 基础运动
控制小车以指定速度运动。速度范围通常为 `0-800`。
- **前进/后退**:
  ```cpp
  motors.moveForward(400);
  motors.moveBackward(400);
  ```
- **左右平移 (Mecanum 特有)**:
  ```cpp
  motors.moveLeft(400);
  motors.moveRight(400);
  ```
- **停止**:
  ```cpp
  motors.stop();
  ```

### 3. 旋转控制
- **原地旋转**:
  `angle` 为正表示顺时针，为负表示逆时针。目前主要用于方向控制，精准角度需配合编码器逻辑。
  ```cpp
  motors.turn(90, 400);  // 顺时针旋转
  motors.turn(-90, 400); // 逆时针旋转
  ```

### 4. 编码器管理
- **获取计数值**:
  获取各轮子自上次重置以来的脉冲数。
  ```cpp
  long fl = motors.getCountFL();
  long fr = motors.getCountFR();
  ```
- **重置**:
  在执行新动作前通常需要重置计数值。
  ```cpp
  motors.resetEncoders();
  ```

### 5. 高级设置
- **直接控制**:
  如果需要实现复杂的自定义路径，可以直接设置四个电机的速度。
  ```cpp
  // 参数: 左前(FL), 右前(FR), 左后(RL), 右后(RR)
  motors.setSpeeds(200, -200, 200, -200); 
  ```

---

## 注意事项
- **I2C 总线**: 本库在 Arduino Giga 上默认使用 `Wire1`。
- **引脚分配**: 请确保 `include/config.h` 中的编码器引脚与实际硬件接线一致。
- **中断限制**: 编码器 A 相必须连接到支持外部中断的引脚。
