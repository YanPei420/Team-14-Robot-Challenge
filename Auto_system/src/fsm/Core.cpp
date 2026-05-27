#include "../RobotFSM.h"

#include "FSMconfig.h"

RobotFSM::RobotFSM(RobotDrive& robot)
    : robot_(robot),
      navigator_(nullptr),
      safetyState_(SafetyState::Normal),
      missionState_(MissionState::Base),
      baseState_(BaseState::Idle),
      gridState_(GridState::ExploreGrid),
      exitBaseState_(ExitBaseState::RequestClearance),
      exploreState_(ExploreState::DriveGrid),
      alignState_(AlignState::SearchRFID),
      plantState_(PlantState::OpenHopper),
      returnState_(ReturnState::NavigateToAirlock),
      emergencyReturn_(false),
      stateStartedAt_(0),
      arenaEnteredAt_(0),
      seedsPlanted_(0),
      pendingTagValid_(false),
      pendingTag_{{0}, false},
      stateNameBuffer_{0}
{
}

void RobotFSM::begin()
{
    safetyState_ = SafetyState::Normal;
    transitionMission(MissionState::Base);
}

void RobotFSM::update()
{
    if (safetyState_ == SafetyState::EmergencyStop)
    {
        return;
    }

    updateNormal();
}

void RobotFSM::setNavigator(RobotNavigation::Navigator* navigator)
{
    navigator_ = navigator;
}

void RobotFSM::updateNormal()
{
    if (isArenaMissionState() && arenaEnteredAt_ != 0 &&
        millis() - arenaEnteredAt_ >= RobotFSMConfig::ARENA_TIME_LIMIT_MS)
    {
        transitionToReturnHome(false);
    }

    switch (missionState_)
    {
        case MissionState::Base:
            updateBase();
            break;

        case MissionState::Grid:
            updateGrid();
            break;

        case MissionState::Stranded:
            break;

        case MissionState::Revived:
            updateRevived();
            break;
    }
}

void RobotFSM::updateRevived()
{
    if (stateElapsed(RobotFSMConfig::REVIVE_PAUSE_MS))
    {
        transitionToReturnHome(false);
    }
}

SafetyState RobotFSM::safetyState() const
{
    return safetyState_;
}

MissionState RobotFSM::missionState() const
{
    return missionState_;
}

BaseState RobotFSM::baseState() const
{
    return baseState_;
}

GridState RobotFSM::gridState() const
{
    return gridState_;
}

ExitBaseState RobotFSM::exitBaseState() const
{
    return exitBaseState_;
}

ExploreState RobotFSM::exploreState() const
{
    return exploreState_;
}

AlignState RobotFSM::alignState() const
{
    return alignState_;
}

PlantState RobotFSM::plantState() const
{
    return plantState_;
}

ReturnState RobotFSM::returnState() const
{
    return returnState_;
}

bool RobotFSM::isEmergencyStop() const
{
    return safetyState_ == SafetyState::EmergencyStop;
}

bool RobotFSM::isEmergencyReturn() const
{
    return safetyState_ == SafetyState::Normal &&
           missionState_ == MissionState::Base &&
           baseState_ == BaseState::ReturnHome &&
           emergencyReturn_;
}

uint8_t RobotFSM::seedsPlanted() const
{
    return seedsPlanted_;
}

uint8_t RobotFSM::seedsRemaining() const
{
    return MAX_SEEDS - seedsPlanted_;
}

bool RobotFSM::hasPendingTag() const
{
    return pendingTagValid_;
}

RfidTag RobotFSM::pendingTag() const
{
    return pendingTag_;
}

void RobotFSM::reenterCurrentLeafState()
{
    switch (missionState_)
    {
        case MissionState::Base:
            switch (baseState_)
            {
                case BaseState::Idle:
                case BaseState::InsideBase:
                    stopRobot();
                    break;

                case BaseState::ExitBase:
                    enterExitBaseState();
                    break;

                case BaseState::ReturnHome:
                    enterReturnState();
                    break;
            }
            break;

        case MissionState::Grid:
            switch (gridState_)
            {
                case GridState::ExploreGrid:
                    enterExploreState();
                    break;

                case GridState::Align:
                    enterAlignState();
                    break;

                case GridState::Plant:
                    enterPlantState();
                    break;
            }
            break;

        case MissionState::Stranded:
        case MissionState::Revived:
            stopRobot();
            break;
    }
}

void RobotFSM::stopRobot()
{
    robot_.stop_all();
}

void RobotFSM::clearPendingTag()
{
    pendingTagValid_ = false;
    pendingTag_.coordinate[0] = '\0';
    pendingTag_.fertile = false;
}

bool RobotFSM::stateElapsed(uint32_t durationMs) const
{
    return millis() - stateStartedAt_ >= durationMs;
}

bool RobotFSM::isInBaseState() const
{
    return missionState_ == MissionState::Base &&
           (baseState_ == BaseState::Idle ||
            baseState_ == BaseState::InsideBase ||
            (baseState_ == BaseState::ExitBase &&
             exitBaseState_ == ExitBaseState::RequestClearance));
}

bool RobotFSM::isArenaMissionState() const
{
    return missionState_ == MissionState::Grid;
}
