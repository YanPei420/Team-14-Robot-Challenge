#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include <Arduino.h>
#include "LidarConfig.h"

// TF-Luna 9-byte UART frame:
// [0x59][0x59][dist_L][dist_H][amp_L][amp_H][temp_L][temp_H][checksum]

class LidarSensor
{
private:
    HardwareSerial& serial;

    int16_t       distanceCM;
    uint16_t      strength;
    float         temperature;
    unsigned long lastUpdateMs;
    uint32_t      validFrameCount;
    uint32_t      checksumErrorCount;
    uint32_t      lowStrengthCount;
    uint32_t      droppedByteCount;

public:
    LidarSensor(HardwareSerial& serialPort);

    void begin();

    // Poll the UART buffer and parse one frame.
    // Returns true if a new valid frame was received this call.
    bool update();

    int16_t       getDistanceCM();
    uint16_t      getStrength();
    float         getTemperature();
    unsigned long getLastUpdateMs();
    uint32_t      getValidFrameCount();
    uint32_t      getChecksumErrorCount();
    uint32_t      getLowStrengthCount();
    uint32_t      getDroppedByteCount();

    // True once at least one valid reading has been received
    bool isValid();
};

#endif
