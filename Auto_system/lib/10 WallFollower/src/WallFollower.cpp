#include "WallFollower.h"

WallFollower::WallFollower(
    MotoronDrive& drive,
    LidarSensor& leftLidar,
    LidarSensor& rightLidar,
    const WallFollowerConfig& followerConfig
)
    : robot(drive),
      left(leftLidar),
      right(rightLidar),
      config(followerConfig),
      lastControlMs(0),
      lastTurnValue(followerConfig.searchTurnSpeed),
      currentMode(WallFollowerMode::SearchWall)
{
}

void WallFollower::reset()
{
    lastControlMs = 0;
    lastTurnValue = config.searchTurnSpeed;
    currentMode = WallFollowerMode::SearchWall;
}

bool WallFollower::lidarFresh(LidarSensor& lidar) const
{
    return lidar.isValid() &&
        millis() - lidar.getLastUpdateMs() <= config.lidarFreshMs;
}

bool WallFollower::wallVisible(LidarSensor& lidar) const
{
    return lidarFresh(lidar)
        && lidar.getDistanceCM() > 0
        && lidar.getDistanceCM() <= static_cast<int16_t>(config.wallVisibleMaxCm);
}

int16_t WallFollower::clampTurn(float turn) const
{
    if (turn > config.maxTurnSpeed)
    {
        return config.maxTurnSpeed;
    }

    if (turn < -config.maxTurnSpeed)
    {
        return -config.maxTurnSpeed;
    }

    return static_cast<int16_t>(turn);
}

void WallFollower::update(int16_t speed)
{
    const uint32_t now = millis();

    if (now - lastControlMs < config.controlIntervalMs)
    {
        return;
    }

    lastControlMs = now;

    const bool leftVisible = wallVisible(left);
    const bool rightVisible = wallVisible(right);

    if (leftVisible && rightVisible)
    {
        const float leftDistance = static_cast<float>(left.getDistanceCM());
        const float rightDistance = static_cast<float>(right.getDistanceCM());
        const float error = rightDistance - leftDistance;

        lastTurnValue = (error > config.deadbandCm || error < -config.deadbandCm)
            ? clampTurn(error * config.centerKp)
            : 0;
        currentMode = WallFollowerMode::CenterBetweenWalls;

        robot.drive(speed, 0, lastTurnValue);
        return;
    }

    if (leftVisible)
    {
        const float distance = static_cast<float>(left.getDistanceCM());
        const float error = config.targetWallDistanceCm - distance;

        lastTurnValue = (error > config.deadbandCm || error < -config.deadbandCm)
            ? clampTurn(error * config.singleWallKp)
            : 0;
        currentMode = WallFollowerMode::FollowLeft;

        robot.drive(speed, 0, lastTurnValue);
        return;
    }

    if (rightVisible)
    {
        const float distance = static_cast<float>(right.getDistanceCM());
        const float error = distance - config.targetWallDistanceCm;

        lastTurnValue = (error > config.deadbandCm || error < -config.deadbandCm)
            ? clampTurn(error * config.singleWallKp)
            : 0;
        currentMode = WallFollowerMode::FollowRight;

        robot.drive(speed, 0, lastTurnValue);
        return;
    }

    lastTurnValue = lastTurnValue < 0
        ? -config.searchTurnSpeed
        : config.searchTurnSpeed;
    currentMode = WallFollowerMode::SearchWall;
    robot.drive(speed / 2, 0, lastTurnValue);
}

bool WallFollower::leftWallVisible() const
{
    return wallVisible(left);
}

bool WallFollower::rightWallVisible() const
{
    return wallVisible(right);
}

int16_t WallFollower::lastTurn() const
{
    return lastTurnValue;
}

WallFollowerMode WallFollower::mode() const
{
    return currentMode;
}
