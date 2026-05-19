#include "../RobotFSM.h"

void RobotFSM::emergencyWarningReceived()
{
    if (!isInBaseState() && missionState_ != MissionState::Stranded &&
        safetyState_ == SafetyState::Normal)
    {
        transitionToReturnHome(true);
    }
}

void RobotFSM::markStranded()
{
    if (!isInBaseState() && safetyState_ == SafetyState::Normal)
    {
        transitionMission(MissionState::Stranded);
    }
}

void RobotFSM::reviveFromStranded()
{
    if (missionState_ == MissionState::Stranded &&
        safetyState_ == SafetyState::Normal)
    {
        transitionMission(MissionState::Revived);
    }
}

void RobotFSM::triggerEmergencyStop()
{
    if (safetyState_ != SafetyState::EmergencyStop)
    {
        transitionSafety(SafetyState::EmergencyStop);
    }
}

void RobotFSM::clearEmergencyStop()
{
    if (safetyState_ == SafetyState::EmergencyStop)
    {
        transitionSafety(SafetyState::Normal);
        logState();
    }
}

void RobotFSM::transitionSafety(SafetyState nextState)
{
    if (safetyState_ == nextState)
    {
        return;
    }

    safetyState_ = nextState;
    stateStartedAt_ = millis();

    if (safetyState_ == SafetyState::EmergencyStop)
    {
        stopRobot();
    }
    else
    {
        reenterCurrentLeafState();
    }

    logState();
}

void RobotFSM::transitionMission(MissionState nextState)
{
    if (missionState_ == nextState)
    {
        enterMissionState();
        logState();
        return;
    }

    missionState_ = nextState;
    stateStartedAt_ = millis();

    enterMissionState();
    logState();
}

void RobotFSM::transitionBase(BaseState nextState)
{
    const bool missionChanged = missionState_ != MissionState::Base;

    if (missionState_ != MissionState::Base)
    {
        missionState_ = MissionState::Base;
    }

    if (baseState_ == nextState)
    {
        if (missionChanged)
        {
            stateStartedAt_ = millis();
        }

        enterBaseState();
        logState();
        return;
    }

    baseState_ = nextState;
    stateStartedAt_ = millis();
    enterBaseState();
    logState();
}

void RobotFSM::transitionGrid(GridState nextState)
{
    const bool missionChanged = missionState_ != MissionState::Grid;

    if (missionState_ != MissionState::Grid)
    {
        missionState_ = MissionState::Grid;
    }

    if (gridState_ == nextState)
    {
        if (missionChanged)
        {
            stateStartedAt_ = millis();
        }

        enterGridState();
        logState();
        return;
    }

    gridState_ = nextState;
    stateStartedAt_ = millis();
    enterGridState();
    logState();
}

void RobotFSM::transitionExitBase(ExitBaseState nextState)
{
    if (exitBaseState_ == nextState)
    {
        return;
    }

    exitBaseState_ = nextState;
    stateStartedAt_ = millis();
    enterExitBaseState();
    logState();
}

void RobotFSM::transitionExplore(ExploreState nextState)
{
    if (exploreState_ == nextState)
    {
        return;
    }

    exploreState_ = nextState;
    stateStartedAt_ = millis();
    enterExploreState();
    logState();
}

void RobotFSM::transitionAlign(AlignState nextState)
{
    if (alignState_ == nextState)
    {
        return;
    }

    alignState_ = nextState;
    stateStartedAt_ = millis();
    enterAlignState();
    logState();
}

void RobotFSM::transitionPlant(PlantState nextState)
{
    if (plantState_ == nextState)
    {
        return;
    }

    plantState_ = nextState;
    stateStartedAt_ = millis();
    enterPlantState();
    logState();
}

void RobotFSM::transitionReturn(ReturnState nextState)
{
    if (returnState_ == nextState)
    {
        return;
    }

    returnState_ = nextState;
    stateStartedAt_ = millis();
    enterReturnState();
    logState();
}

void RobotFSM::transitionToReturnHome(bool emergencyReturn)
{
    emergencyReturn_ = emergencyReturn_ || emergencyReturn;
    transitionBase(BaseState::ReturnHome);
}

void RobotFSM::enterMissionState()
{
    switch (missionState_)
    {
        case MissionState::Base:
            enterBaseState();
            break;

        case MissionState::Grid:
            gridState_ = GridState::ExploreGrid;
            enterGridState();
            break;

        case MissionState::Stranded:
        case MissionState::Revived:
            stopRobot();
            break;
    }
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
