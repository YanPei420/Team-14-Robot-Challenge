#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "LidarConfig.h"

// TF-Luna I2C register map (Benewake protocol)
// Reading 6 bytes from REG_DIST_L returns: dist_L, dist_H,
// strength_L, strength_H, temp_L, temp_H
#define LIDAR_REG_DIST_L 0x00

class LidarSensor
{
private:
    TwoWire& wire;
    uint8_t address;

    int16_t distanceCM;
    uint16_t strength;
    int16_t temperature;

    bool parseFrame(uint8_t* buf);

public:
    LidarSensor(TwoWire& wireBus, uint8_t sensorAddress);

    void begin();

    // Request and parse one measurement frame.
    // Returns true if the frame was valid and strength is sufficient.
    bool update();

    // Last valid distance in cm. Returns LIDAR_INVALID_DISTANCE if not yet valid.
    int16_t getDistanceCM();

    // Raw signal strength of the last measurement
    uint16_t getStrength();

    // Chip temperature in 0.01 °C units (divide by 100 for °C)
    int16_t getTemperature();

    // True if the last update() returned a valid reading
    bool isValid();
};

#endif
