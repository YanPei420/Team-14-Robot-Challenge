#include "../RobotFSM.h"

#include "Config.h"

void RobotFSM::updateBase()
{
    switch (baseState_)
    {
        case BaseState::Idle:
        case BaseState::InsideBase:
            break;

        case BaseState::ExitBase:
            updateExitBase();
            break;

        case BaseState::ReturnHome:
            updateReturnHome();
            break;
    }
}

void RobotFSM::updateExitBase()
{
    switch (exitBaseState_)
    {
        case ExitBaseState::RequestClearance:
        case ExitBaseState::LineFollowToDoor:
        case ExitBaseState::WaitForDoor:
        case ExitBaseState::TraverseTunnel:
            break;
    }
}

void RobotFSM::updateReturnHome()
{
    switch (returnState_)
    {
        case ReturnState::NavigateToAirlock:
        case ReturnState::WaitForEntryDoor:
        case ReturnState::TraverseTunnel:
            break;
    }
}

void RobotFSM::startMission()
{
    if (safetyState_ != SafetyState::Normal)
    {
        return;
    }

    if (missionState_ == MissionState::Base &&
        (baseState_ == BaseState::Idle || baseState_ == BaseState::InsideBase))
    {
        seedsPlanted_ = 0;
        arenaEnteredAt_ = 0;
        emergencyReturn_ = false;
        clearPendingTag();
        transitionBase(BaseState::ExitBase);
    }
}

void RobotFSM::exitClearanceReceived()
{
    if (missionState_ == MissionState::Base &&
        baseState_ == BaseState::ExitBase &&
        exitBaseState_ == ExitBaseState::RequestClearance)
    {
        transitionExitBase(ExitBaseState::LineFollowToDoor);
    }
}

void RobotFSM::exitDoorDetected()
{
    if (missionState_ == MissionState::Base &&
        baseState_ == BaseState::ExitBase &&
        exitBaseState_ == ExitBaseState::LineFollowToDoor)
    {
        transitionExitBase(ExitBaseState::WaitForDoor);
    }
}

void RobotFSM::exitDoorOpened()
{
    if (missionState_ == MissionState::Base &&
        baseState_ == BaseState::ExitBase &&
        exitBaseState_ == ExitBaseState::WaitForDoor)
    {
        transitionExitBase(ExitBaseState::TraverseTunnel);
    }
}

void RobotFSM::mainArenaReached()
{
    if (missionState_ == MissionState::Base &&
        baseState_ == BaseState::ExitBase &&
        exitBaseState_ == ExitBaseState::TraverseTunnel)
    {
        arenaEnteredAt_ = millis();
        transitionMission(MissionState::Grid);
    }
}

void RobotFSM::entryAirlockReached()
{
    if (missionState_ == MissionState::Base &&
        baseState_ == BaseState::ReturnHome &&
        returnState_ == ReturnState::NavigateToAirlock)
    {
        transitionReturn(ReturnState::WaitForEntryDoor);
    }
}

void RobotFSM::entryDoorOpened()
{
    if (missionState_ == MissionState::Base &&
        baseState_ == BaseState::ReturnHome &&
        returnState_ == ReturnState::WaitForEntryDoor)
    {
        transitionReturn(ReturnState::TraverseTunnel);
    }
}

void RobotFSM::baseReached()
{
    if (missionState_ == MissionState::Base &&
        baseState_ == BaseState::ReturnHome)
    {
        emergencyReturn_ = false;
        transitionBase(BaseState::InsideBase);
    }
}

void RobotFSM::enterBaseState()
{
    switch (baseState_)
    {
        case BaseState::Idle:
        case BaseState::InsideBase:
            stopRobot();
            break;

        case BaseState::ExitBase:
            exitBaseState_ = ExitBaseState::RequestClearance;
            enterExitBaseState();
            break;

        case BaseState::ReturnHome:
            returnState_ = ReturnState::NavigateToAirlock;
            enterReturnState();
            break;
    }
}

void RobotFSM::enterExitBaseState()
{
    switch (exitBaseState_)
    {
        case ExitBaseState::RequestClearance:
        case ExitBaseState::WaitForDoor:
            stopRobot();
            break;

        case ExitBaseState::LineFollowToDoor:
            robot_.forward(RobotFSMConfig::LINE_FOLLOW_SPEED);
            break;

        case ExitBaseState::TraverseTunnel:
            robot_.forward(RobotFSMConfig::TUNNEL_SPEED);
            break;
    }
}

void RobotFSM::enterReturnState()
{
    switch (returnState_)
    {
        case ReturnState::NavigateToAirlock:
            robot_.backward(RobotFSMConfig::RETURN_SPEED);
            break;

        case ReturnState::WaitForEntryDoor:
            stopRobot();
            break;

        case ReturnState::TraverseTunnel:
            robot_.forward(RobotFSMConfig::TUNNEL_SPEED);
            break;
    }
}
