#pragma once

#include "GridMap.h"

namespace RobotNavigation
{
class PoseEstimator
{
public:
    void reset();
    void observeRfid(GridCoordinate coordinate);
    void setTarget(GridCoordinate target);

    bool hasPosition() const;
    GridCoordinate position() const;
    bool hasTarget() const;
    GridCoordinate target() const;

private:
    bool hasPosition_ = false;
    bool hasTarget_ = false;
    GridCoordinate position_;
    GridCoordinate target_;
};
} // namespace RobotNavigation
