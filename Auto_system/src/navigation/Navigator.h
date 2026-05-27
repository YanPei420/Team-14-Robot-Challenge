#pragma once

#include <Arduino.h>

#include "../RobotDrive.h"
#include "GridMap.h"
#include "PoseEstimator.h"

class IRSensor;

namespace RobotNavigation
{
struct LineReading
{
    bool visible = false;
    int16_t error = 0;
    uint16_t contrast = 0;
};

struct DriveCommand
{
    int16_t vx = 0;
    int16_t vy = 0;
    int16_t w = 0;
};

class Navigator
{
public:
    void begin();
    void resetMission();

    void observeLine(IRSensor& sensors);
    void observeObstacleDistance(int16_t distanceCm);
    bool observeRfidTag(
        const String& uid,
        char* coordinateText,
        size_t coordinateTextSize,
        bool& fertile
    );
    bool observeRemoteRfidCoordinate(const char* coordinateText, bool fertile);
    void markCurrentCellPlanted();

    void driveExitLine(RobotDrive& drive);
    void driveGridExplore(RobotDrive& drive);
    void driveFineAdjust(RobotDrive& drive);
    void driveReturnHome(RobotDrive& drive);
    void driveTunnel(RobotDrive& drive);
    void stop(RobotDrive& drive);

    bool chooseNextTarget(GridCoordinate& target);
    const LineReading& lineReading() const;
    const GridMap& map() const;
    GridMap& map();
    const PoseEstimator& pose() const;

private:
    static constexpr int16_t LINE_FOLLOW_SPEED = 120;
    static constexpr int16_t GRID_EXPLORE_SPEED = 100;
    static constexpr int16_t ALIGN_SPEED = 70;
    static constexpr int16_t RETURN_SPEED = 130;
    static constexpr int16_t TUNNEL_SPEED = 90;
    static constexpr int16_t SEARCH_TURN_SPEED = 120;
    static constexpr int16_t MAX_TURN_SPEED = 800;
    static constexpr int16_t SENSOR_STEP = 1000;
    static constexpr uint16_t LINE_MIN_CONTRAST = 50;
    static constexpr float TURN_KP = 0.08f;
    static constexpr int16_t OBSTACLE_STOP_CM = 12;

    GridMap map_;
    PoseEstimator pose_;
    LineReading line_;
    int16_t lastTurn_ = 0;
    int16_t obstacleDistanceCm_ = -1;

    DriveCommand lineFollowCommand(int16_t speed);
    void applyCommand(RobotDrive& drive, DriveCommand command);
    bool obstacleTooClose() const;
};
} // namespace RobotNavigation
