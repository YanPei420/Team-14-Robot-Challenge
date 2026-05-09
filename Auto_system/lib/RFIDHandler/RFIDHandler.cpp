#include "RFIDHandler.h"

// 假设使用 SoftwareSerial 或 HardwareSerial 进行通信
// 这里使用 Serial2 作为占位
RFIDHandler::RFIDHandler() : _lastTag("") {}

void RFIDHandler::begin() {
    Serial2.begin(9600); // 通常 RFID 模块使用 9600 波特率
    Serial.println("RFIDHandler 初始化完成 (Serial2)。");
}

bool RFIDHandler::available() {
    if (Serial2.available() > 0) {
        // 读取 RFID 数据流并解析 ID
        // 此处为伪代码，需根据实际使用的 RFID 模块型号调整协议
        _lastTag = "TAG_ID_001"; 
        return true;
    }
    return false;
}

String RFIDHandler::getTagID() {
    return _lastTag;
}
