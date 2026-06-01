#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include <Arduino.h>

#include "LidarSensor.h"
#include "MotoronDrive.h"

enum class WallFollowerMode : uint8_t
{
    SearchWall,
    FollowLeft,
    FollowRight,
    CenterBetweenWalls
};

struct WallFollowerConfig
{
    uint32_t controlIntervalMs = 30;
    uint32_t lidarFreshMs = 350;
    float targetWallDistanceCm = 12.0f;
    float wallVisibleMaxCm = 60.0f;
    float deadbandCm = 1.5f;
    float centerKp = 35.0f;
    float singleWallKp = 45.0f;
    int16_t searchTurnSpeed = 140;
    int16_t maxTurnSpeed = 260;
};

class WallFollower
{
private:
    MotoronDrive& robot;
    LidarSensor& left;
    LidarSensor& right;
    WallFollowerConfig config;

    uint32_t lastControlMs;
    int16_t lastTurnValue;
    WallFollowerMode currentMode;

    bool lidarFresh(LidarSensor& lidar) const;
    bool wallVisible(LidarSensor& lidar) const;
    int16_t clampTurn(float turn) const;

public:
    WallFollower(
        MotoronDrive& drive,
        LidarSensor& leftLidar,
        LidarSensor& rightLidar,
        const WallFollowerConfig& followerConfig = WallFollowerConfig()
    );

    void reset();
    void update(int16_t speed);

    bool leftWallVisible() const;
    bool rightWallVisible() const;
    int16_t lastTurn() const;
    WallFollowerMode mode() const;
};

#endif
