# RFIDHandler 库

## 文件夹作用
用于处理 RFID 传感器的数据，识别赛场各个顶点的唯一标识（ID）。

## 主要函数
- `begin()`: 初始化 RFID 模块串口（默认 Serial2，波特率 9600）。
- `available()`: 检测是否有新的 RFID 标签信号。
- `getTagID()`: 获取最近检测到的标签 ID 字符串。

## 注意事项
- 本代码目前为占位实现，RFID 协议通常与具体模块（如 MFRC522 或 PN532）相关，请根据你的硬件替换 `available()` 中的数据解析逻辑。
