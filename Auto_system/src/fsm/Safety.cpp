#include "../RobotFSM.h"

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
