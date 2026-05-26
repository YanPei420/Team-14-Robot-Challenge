#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>
#include "IRConfig.h"

class IRSensor
{
private:
    const uint8_t* pins;
    uint8_t count;

    uint16_t values[IR_SENSOR_COUNT];

public:
    IRSensor(const uint8_t* sensorPins, uint8_t sensorCount);

    void begin();

    // Call each loop iteration to refresh readings
    void update();

    // Raw RC discharge time in microseconds. Timeout returns IR_READ_TIMEOUT_US.
    uint16_t getValue(uint8_t index);
    const uint16_t* getValues();
    uint8_t getCount();
};

#endif
