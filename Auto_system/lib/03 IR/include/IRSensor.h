#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>
#include "IRConfig.h"

class IRSensor
{
private:
    const uint8_t* pins;
    uint8_t count;

    bool readings[IR_SENSOR_COUNT];
    float lastPosition;

    bool sensorActive(uint8_t index);

public:
    IRSensor(const uint8_t* sensorPins, uint8_t sensorCount);

    void begin();

    // Call each loop iteration to refresh readings
    void update();

    // True if sensor[index] sees the line
    bool isDetected(uint8_t index);

    // True if at least one sensor sees the line
    bool isLineDetected();

    // Weighted centroid position: -1.0 (far left) to +1.0 (far right), 0.0 = centered
    // Returns last known position when no sensor sees the line
    float getLinePosition();

    uint8_t getCount();
};

#endif
