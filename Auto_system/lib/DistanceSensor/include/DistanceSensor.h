#pragma once

#include <Arduino.h>
#include <Wire.h>

class DistanceSensor
{
private:

    TwoWire* wire;

    uint8_t address;

public:

    DistanceSensor(
        TwoWire& wireBus,
        uint8_t sensorAddress
    );

    void begin();

    float readDistanceCM();
};