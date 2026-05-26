#include "../../RobotFSM.h"

#include <string.h>

#include "../../navigation/Navigator.h"
#include "../FSMconfig.h"

void RobotFSM::updateGrid()
{
    switch (gridState_)
    {
        case GridState::ExploreGrid:
            updateExploreGrid();
            break;

        case GridState::Align:
            updateAlign();
            break;

        case GridState::Plant:
            updatePlant();
            break;
    }
}

void RobotFSM::updateExploreGrid()
{
    switch (exploreState_)
    {
        case ExploreState::DriveGrid:
            if (navigator_ != nullptr)
            {
                navigator_->driveGridExplore(robot_);
            }
            break;

        case ExploreState::QuerySoilStatus:
            if (!pendingTagValid_ ||
                stateElapsed(RobotFSMConfig::SOIL_QUERY_TIMEOUT_MS))
            {
                clearPendingTag();
                transitionExplore(ExploreState::DriveGrid);
            }
            else if (pendingTag_.fertile && seedsRemaining() > 0)
            {
                transitionGrid(GridState::Align);
            }
            else
            {
                clearPendingTag();
                transitionExplore(ExploreState::DriveGrid);
            }
            break;
    }
}

void RobotFSM::updateAlign()
{
    switch (alignState_)
    {
        case AlignState::SearchRFID:
            if (stateElapsed(RobotFSMConfig::SEARCH_RFID_MS))
            {
                transitionAlign(AlignState::FineAdjustToHole);
            }
            break;

        case AlignState::FineAdjustToHole:
            if (stateElapsed(RobotFSMConfig::FINE_ADJUST_MS))
            {
                transitionGrid(GridState::Plant);
            }
            break;
    }
}

void RobotFSM::updatePlant()
{
    switch (plantState_)
    {
        case PlantState::OpenHopper:
            if (stateElapsed(RobotFSMConfig::OPEN_HOPPER_MS))
            {
                transitionPlant(PlantState::DropSeed);
            }
            break;

        case PlantState::DropSeed:
            if (stateElapsed(RobotFSMConfig::DROP_SEED_MS))
            {
                transitionPlant(PlantState::VerifyDrop);
            }
            break;

        case PlantState::VerifyDrop:
            if (stateElapsed(RobotFSMConfig::VERIFY_DROP_MS))
            {
                plantingMechanismDone();
            }
            break;
    }
}

void RobotFSM::rfidDetected(const char* coordinate, bool fertile)
{
    if (missionState_ != MissionState::Grid ||
        gridState_ != GridState::ExploreGrid ||
        exploreState_ != ExploreState::DriveGrid)
    {
        return;
    }

    strncpy(pendingTag_.coordinate, coordinate, sizeof(pendingTag_.coordinate) - 1);
    pendingTag_.coordinate[sizeof(pendingTag_.coordinate) - 1] = '\0';
    pendingTag_.fertile = fertile;
    pendingTagValid_ = true;

    transitionExplore(ExploreState::QuerySoilStatus);
}

void RobotFSM::plantingMechanismDone()
{
    if (missionState_ != MissionState::Grid ||
        gridState_ != GridState::Plant ||
        plantState_ != PlantState::VerifyDrop)
    {
        return;
    }

    if (seedsPlanted_ < MAX_SEEDS)
    {
        ++seedsPlanted_;
    }

    clearPendingTag();
    if (navigator_ != nullptr)
    {
        navigator_->markCurrentCellPlanted();
    }

    if (seedsRemaining() == 0)
    {
        transitionToReturnHome(false);
    }
    else
    {
        transitionGrid(GridState::ExploreGrid);
    }
}

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

void RobotFSM::enterGridState()
{
    switch (gridState_)
    {
        case GridState::ExploreGrid:
            exploreState_ = ExploreState::DriveGrid;
            enterExploreState();
            break;

        case GridState::Align:
            alignState_ = AlignState::SearchRFID;
            enterAlignState();
            break;

        case GridState::Plant:
            plantState_ = PlantState::OpenHopper;
            enterPlantState();
            break;
    }
}

void RobotFSM::enterExploreState()
{
    switch (exploreState_)
    {
        case ExploreState::DriveGrid:
            if (navigator_ != nullptr)
            {
                navigator_->driveGridExplore(robot_);
            }
            else
            {
                robot_.forward(RobotFSMConfig::GRID_EXPLORE_SPEED);
            }
            break;

        case ExploreState::QuerySoilStatus:
            stopRobot();
            break;
    }
}

void RobotFSM::enterAlignState()
{
    switch (alignState_)
    {
        case AlignState::SearchRFID:
            stopRobot();
            break;

        case AlignState::FineAdjustToHole:
            if (navigator_ != nullptr)
            {
                navigator_->driveFineAdjust(robot_);
            }
            else
            {
                robot_.forward(RobotFSMConfig::ALIGN_SPEED);
            }
            break;
    }
}

void RobotFSM::enterPlantState()
{
    switch (plantState_)
    {
        case PlantState::OpenHopper:
        case PlantState::DropSeed:
        case PlantState::VerifyDrop:
            stopRobot();
            break;
    }
}
