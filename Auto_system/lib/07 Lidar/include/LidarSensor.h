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

    int16_t  distanceCM;
    uint16_t strength;
    float    temperature;

    bool parseFrame(uint8_t* buf);

public:
    LidarSensor(HardwareSerial& serialPort);

    void begin();

    // Read and parse one frame from the serial buffer.
    // Returns true if a valid frame with sufficient signal strength was found.
    bool update();

    int16_t  getDistanceCM();
    uint16_t getStrength();
    float    getTemperature();

    // True if the last update() returned a valid reading
    bool isValid();
};

#endif
