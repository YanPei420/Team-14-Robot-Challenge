#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include <Arduino.h>

class LidarSensor {
public:
    LidarSensor();
    void begin();
    int getDistance();
private:
    // Implementation specific variables
};

#endif
