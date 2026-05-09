# Robot FSM (Finite State Machine)

## 文件夹作用
本文件夹实现了机器人的高层状态机逻辑，用于协调传感器输入、通信命令和物理按键，决定机器人的运行行为。

## 状态定义 (`RobotState`)
- **`STATE_STOPPED`**: 待机/急停状态。电机关闭，LED 红色闪烁。这是启动后的默认状态。
- **`STATE_RUNNING`**: 正常运行状态。执行巡线或探索任务，LED 红色常亮。
- **`STATE_AVOIDING`**: 避障转向状态。当 Lidar 检测到障碍物时进入，执行转向动作后自动返回 `STATE_RUNNING`。
- **`STATE_PLANTING`**: 播种演示状态。执行播种机构动作。

## 核心功能
1. **双重急停**:
   - **物理按钮**: 通过 `PIN_KILL_SWITCH` 实现 Toggle 切换（停 -> 跑 -> 停）。
   - **UDP 指令**: 接收 "STOP" 进入停止态，接收 "START" 进入运行态。
2. **复活/拍打机制**:
   - 按下 `PIN_TAPPING_BUTTON` 时，LED 立即变绿（最高优先级），松开后恢复状态机控制。
3. **视觉反馈**:
   - **红色闪烁**: 停止/急停。
   - **红色常亮**: 运行中。
   - **绿色常亮**: 被拍打（复活中）。

## 使用方法
1. 在 `main.cpp` 中调用 `setupFSM()` 进行初始化。
2. 在 `loop()` 中持续调用 `updateFSM()` 处理逻辑。
