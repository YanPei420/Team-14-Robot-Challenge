#ifndef RFID_HANDLER_H
#define RFID_HANDLER_H

#include <Arduino.h>

/**
 * @brief RFID 传感器处理类
 * 
 * 用于识别赛场各个顶点的唯一 ID，以便向服务器请求位置状态。
 */
class RFIDHandler {
public:
    RFIDHandler();

    /**
     * @brief 初始化 RFID 模块
     */
    void begin();

    /**
     * @brief 检查是否检测到标签
     * @return true 检测到新标签
     */
    bool available();

    /**
     * @brief 获取检测到的标签 ID
     * @return String 标签 ID 字符串
     */
    String getTagID();

private:
    String _lastTag;
};

#endif
