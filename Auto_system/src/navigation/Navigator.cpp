#include "Navigator.h"

#include "IRSensor.h"

namespace RobotNavigation
{
void Navigator::begin()
{
    resetMission();
}

void Navigator::resetMission()
{
    map_.reset();
    pose_.reset();
    line_ = LineReading{};
    lastTurn_ = 0;
    obstacleDistanceCm_ = -1;
}

void Navigator::observeLine(IRSensor& sensors)
{
    uint16_t minValue = IR_READ_TIMEOUT_US;
    uint16_t maxValue = 0;

    for (uint8_t i = 0; i < sensors.getCount(); ++i)
    {
        const uint16_t value = sensors.getValue(i);

        if (value < minValue)
        {
            minValue = value;
        }

        if (value > maxValue)
        {
            maxValue = value;
        }
    }

    line_.contrast = maxValue - minValue;

    if (line_.contrast < LINE_MIN_CONTRAST)
    {
        line_.visible = false;
        line_.error = 0;
        return;
    }

    int32_t weightedSum = 0;
    uint32_t signalSum = 0;
    const int16_t center = static_cast<int16_t>(sensors.getCount() - 1) / 2;

    for (uint8_t i = 0; i < sensors.getCount(); ++i)
    {
        const uint16_t signal = sensors.getValue(i) - minValue;
        const int16_t position =
            (static_cast<int16_t>(i) - center) * SENSOR_STEP;

        weightedSum += static_cast<int32_t>(signal) * position;
        signalSum += signal;
    }

    if (signalSum == 0)
    {
        line_.visible = false;
        line_.error = 0;
        return;
    }

    line_.visible = true;
    line_.error = static_cast<int16_t>(
        weightedSum / static_cast<int32_t>(signalSum)
    );
}

void Navigator::observeObstacleDistance(int16_t distanceCm)
{
    obstacleDistanceCm_ = distanceCm;
}

bool Navigator::observeRfidTag(
    const String& uid,
    char* coordinateText,
    size_t coordinateTextSize,
    bool& fertile
)
{
    GridCoordinate coordinate;
    SoilStatus soil = SoilStatus::Unknown;

    if (!map_.lookupRfidTag(uid, coordinate, soil))
    {
        if (coordinateText != nullptr && coordinateTextSize > 0)
        {
            coordinateText[0] = '\0';
        }

        fertile = false;
        return false;
    }

    map_.markVisited(coordinate);
    map_.markSoil(coordinate, soil);
    pose_.observeRfid(coordinate);

    fertile = soil == SoilStatus::Fertile;
    map_.coordinateToString(coordinate, coordinateText, coordinateTextSize);
    return true;
}

bool Navigator::observeRemoteRfidCoordinate(
    const char* coordinateText,
    bool fertile
)
{
    GridCoordinate coordinate;
    if (!map_.parseCoordinate(coordinateText, coordinate))
    {
        return false;
    }

    const SoilStatus soil = fertile ? SoilStatus::Fertile : SoilStatus::Infertile;
    map_.markVisited(coordinate);
    map_.markSoil(coordinate, soil);
    pose_.observeRfid(coordinate);
    return true;
}

void Navigator::markCurrentCellPlanted()
{
    if (pose_.hasPosition())
    {
        map_.markPlanted(pose_.position());
    }
}

void Navigator::driveExitLine(RobotDrive& drive)
{
    applyCommand(drive, lineFollowCommand(LINE_FOLLOW_SPEED));
}

void Navigator::driveGridExplore(RobotDrive& drive)
{
    GridCoordinate target;
    if (chooseNextTarget(target))
    {
        pose_.setTarget(target);
    }

    applyCommand(drive, lineFollowCommand(GRID_EXPLORE_SPEED));
}

void Navigator::driveFineAdjust(RobotDrive& drive)
{
    applyCommand(drive, {ALIGN_SPEED, 0, 0});
}

void Navigator::driveReturnHome(RobotDrive& drive)
{
    applyCommand(drive, {-RETURN_SPEED, 0, 0});
}

void Navigator::driveTunnel(RobotDrive& drive)
{
    applyCommand(drive, {TUNNEL_SPEED, 0, 0});
}

void Navigator::stop(RobotDrive& drive)
{
    drive.stop_all();
}

bool Navigator::chooseNextTarget(GridCoordinate& target)
{
    return map_.chooseNextTarget(target);
}

const LineReading& Navigator::lineReading() const
{
    return line_;
}

const GridMap& Navigator::map() const
{
    return map_;
}

GridMap& Navigator::map()
{
    return map_;
}

const PoseEstimator& Navigator::pose() const
{
    return pose_;
}

DriveCommand Navigator::lineFollowCommand(int16_t speed)
{
    if (obstacleTooClose())
    {
        return {};
    }

    if (line_.visible)
    {
        int32_t turn = static_cast<int32_t>(line_.error * TURN_KP);

        if (turn > MAX_TURN_SPEED)
        {
            turn = MAX_TURN_SPEED;
        }
        else if (turn < -MAX_TURN_SPEED)
        {
            turn = -MAX_TURN_SPEED;
        }

        lastTurn_ = static_cast<int16_t>(turn);
        return {speed, 0, lastTurn_};
    }

    if (lastTurn_ < 0)
    {
        return {0, 0, -SEARCH_TURN_SPEED};
    }

    return {0, 0, SEARCH_TURN_SPEED};
}

void Navigator::applyCommand(RobotDrive& drive, DriveCommand command)
{
    if (command.vx == 0 && command.vy == 0 && command.w == 0)
    {
        drive.stop_all();
        return;
    }

    drive.drive(command.vx, command.vy, command.w);
}

bool Navigator::obstacleTooClose() const
{
    return obstacleDistanceCm_ > 0 && obstacleDistanceCm_ <= OBSTACLE_STOP_CM;
}
} // namespace RobotNavigation
