#include "PoseEstimator.h"

namespace RobotNavigation
{
void PoseEstimator::reset()
{
    hasPosition_ = false;
    hasTarget_ = false;
    position_ = GridCoordinate{};
    target_ = GridCoordinate{};
}

void PoseEstimator::observeRfid(GridCoordinate coordinate)
{
    if (!coordinate.isValid())
    {
        return;
    }

    position_ = coordinate;
    hasPosition_ = true;
}

void PoseEstimator::setTarget(GridCoordinate target)
{
    if (!target.isValid())
    {
        hasTarget_ = false;
        return;
    }

    target_ = target;
    hasTarget_ = true;
}

bool PoseEstimator::hasPosition() const
{
    return hasPosition_;
}

GridCoordinate PoseEstimator::position() const
{
    return position_;
}

bool PoseEstimator::hasTarget() const
{
    return hasTarget_;
}

GridCoordinate PoseEstimator::target() const
{
    return target_;
}
} // namespace RobotNavigation
