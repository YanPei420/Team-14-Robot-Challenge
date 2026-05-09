#include "LidarSensor.h"

LidarSensor::LidarSensor() : _distance(0), _amplitude(0), _temperature(0.0) {}

void LidarSensor::begin() {
    // TF-Luna 默认波特率为 115200
    Serial1.begin(115200);
    Serial.println("LidarSensor (TF-Luna) 已初始化。");
}

void LidarSensor::update() {
    // TF-Luna 数据包长度为 9 字节
    if (Serial1.available() >= 9) {
        uint8_t raw[9];
        // 尝试寻找帧头 0x59 0x59
        if (Serial1.peek() != 0x59) {
            Serial1.read(); // 丢弃不正确的字节
            return;
        }

        Serial1.readBytes(raw, 9);

        // 校验帧头
        if (raw[0] == 0x59 && raw[1] == 0x59) {
            _distance    = raw[2] + raw[3] * 256;
            _amplitude   = raw[4] + raw[5] * 256;
            _temperature = (raw[6] + raw[7] * 256) / 100.0;
        }
    }
}

int LidarSensor::getDistance() const {
    return _distance;
}

int LidarSensor::getAmplitude() const {
    return _amplitude;
}

float LidarSensor::getTemperature() const {
    return _temperature;
}

bool LidarSensor::isReliable() const {
    // 根据测试代码，振幅 < 100 被认为不可靠
    return _amplitude >= 100;
}
