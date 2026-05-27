#include "../RobotFSM.h"

#include <stdio.h>

const char* RobotFSM::stateName() const
{
    if (safetyState_ == SafetyState::EmergencyStop)
    {
        return "Safety/EmergencyStop";
    }

    switch (missionState_)
    {
        case MissionState::Base:
            switch (baseState_)
            {
                case BaseState::ExitBase:
                    snprintf(
                        stateNameBuffer_,
                        sizeof(stateNameBuffer_),
                        "Normal/Base/ExitBase/%s",
                        exitBaseStateName(exitBaseState_)
                    );
                    return stateNameBuffer_;

                case BaseState::ReturnHome:
                    snprintf(
                        stateNameBuffer_,
                        sizeof(stateNameBuffer_),
                        "Normal/Base/%s/%s",
                        emergencyReturn_ ? "EmergencyReturn" : "ReturnHome",
                        returnStateName(returnState_)
                    );
                    return stateNameBuffer_;

                case BaseState::Idle:
                case BaseState::InsideBase:
                    snprintf(
                        stateNameBuffer_,
                        sizeof(stateNameBuffer_),
                        "Normal/Base/%s",
                        baseStateName(baseState_)
                    );
                    return stateNameBuffer_;
            }
            break;

        case MissionState::Grid:
            switch (gridState_)
            {
                case GridState::ExploreGrid:
                    snprintf(
                        stateNameBuffer_,
                        sizeof(stateNameBuffer_),
                        "Normal/Grid/ExploreGrid/%s",
                        exploreStateName(exploreState_)
                    );
                    return stateNameBuffer_;

                case GridState::Align:
                    snprintf(
                        stateNameBuffer_,
                        sizeof(stateNameBuffer_),
                        "Normal/Grid/Align/%s",
                        alignStateName(alignState_)
                    );
                    return stateNameBuffer_;

                case GridState::Plant:
                    snprintf(
                        stateNameBuffer_,
                        sizeof(stateNameBuffer_),
                        "Normal/Grid/Plant/%s",
                        plantStateName(plantState_)
                    );
                    return stateNameBuffer_;
            }
            break;

        case MissionState::Stranded:
        case MissionState::Revived:
            snprintf(
                stateNameBuffer_,
                sizeof(stateNameBuffer_),
                "Normal/%s",
                missionStateName(missionState_)
            );
            return stateNameBuffer_;
    }

    return "Unknown";
}

const char* RobotFSM::missionStateName(MissionState state) const
{
    switch (state)
    {
        case MissionState::Base:
            return "Base";
        case MissionState::Grid:
            return "Grid";
        case MissionState::Stranded:
            return "Stranded";
        case MissionState::Revived:
            return "Revived";
    }

    return "Unknown";
}

const char* RobotFSM::baseStateName(BaseState state) const
{
    switch (state)
    {
        case BaseState::Idle:
            return "Idle";
        case BaseState::ExitBase:
            return "ExitBase";
        case BaseState::ReturnHome:
            return "ReturnHome";
        case BaseState::InsideBase:
            return "InsideBase";
    }

    return "Unknown";
}

const char* RobotFSM::exitBaseStateName(ExitBaseState state) const
{
    switch (state)
    {
        case ExitBaseState::RequestClearance:
            return "RequestClearance";
        case ExitBaseState::LineFollowToDoor:
            return "LineFollowToDoor";
        case ExitBaseState::WaitForDoor:
            return "WaitForDoor";
        case ExitBaseState::TraverseTunnel:
            return "TraverseTunnel";
    }

    return "Unknown";
}

const char* RobotFSM::exploreStateName(ExploreState state) const
{
    switch (state)
    {
        case ExploreState::DriveGrid:
            return "DriveGrid";
        case ExploreState::QuerySoilStatus:
            return "QuerySoilStatus";
    }

    return "Unknown";
}

const char* RobotFSM::alignStateName(AlignState state) const
{
    switch (state)
    {
        case AlignState::SearchRFID:
            return "SearchRFID";
        case AlignState::FineAdjustToHole:
            return "FineAdjustToHole";
    }

    return "Unknown";
}

const char* RobotFSM::plantStateName(PlantState state) const
{
    switch (state)
    {
        case PlantState::OpenHopper:
            return "OpenHopper";
        case PlantState::DropSeed:
            return "DropSeed";
        case PlantState::VerifyDrop:
            return "VerifyDrop";
    }

    return "Unknown";
}

const char* RobotFSM::returnStateName(ReturnState state) const
{
    switch (state)
    {
        case ReturnState::NavigateToAirlock:
            return "NavigateToAirlock";
        case ReturnState::WaitForEntryDoor:
            return "WaitForEntryDoor";
        case ReturnState::TraverseTunnel:
            return "TraverseTunnel";
    }

    return "Unknown";
}

void RobotFSM::logState() const
{
    Serial.print("HFSM -> ");
    Serial.println(stateName());
}
