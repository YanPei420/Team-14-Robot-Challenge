#include "../RobotFSM.h"

#include "Config.h"

RobotFSM::RobotFSM(MotoronDrive& robot)
    : robot_(robot),
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
