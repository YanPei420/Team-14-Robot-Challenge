#pragma once

#include <Arduino.h>

#include "./MotoronDrive.h"

enum class SafetyState : uint8_t
{
    Normal,
    EmergencyStop
};

enum class MissionState : uint8_t
{
    Base,
    Grid,
    Stranded,
    Revived
};

enum class BaseState : uint8_t
{
    Idle,
    ExitBase,
    ReturnHome,
    InsideBase
};

enum class GridState : uint8_t
{
    ExploreGrid,
    Align,
    Plant
};

enum class ExitBaseState : uint8_t
{
    RequestClearance,
    LineFollowToDoor,
    WaitForDoor,
    TraverseTunnel
};

enum class ExploreState : uint8_t
{
    DriveGrid,
    QuerySoilStatus
};

enum class AlignState : uint8_t
{
    SearchRFID,
    FineAdjustToHole
};

enum class PlantState : uint8_t
{
    OpenHopper,
    DropSeed,
    VerifyDrop
};

enum class ReturnState : uint8_t
{
    NavigateToAirlock,
    WaitForEntryDoor,
    TraverseTunnel
};

struct RfidTag
{
    char coordinate[4];
    bool fertile;
};

class RobotFSM
{
public:
    explicit RobotFSM(MotoronDrive& robot);

    void begin();
    void update();

    void startMission();
    void exitClearanceReceived();
    void exitDoorDetected();
    void exitDoorOpened();
    void mainArenaReached();
    void rfidDetected(const char* coordinate, bool fertile);
    void plantingMechanismDone();
    void entryAirlockReached();
    void entryDoorOpened();
    void baseReached();
    void emergencyWarningReceived();
    void markStranded();
    void reviveFromStranded();
    void triggerEmergencyStop();
    void clearEmergencyStop();

    SafetyState safetyState() const;
    MissionState missionState() const;
    BaseState baseState() const;
    GridState gridState() const;
    const char* stateName() const;
    bool isEmergencyStop() const;
    bool isEmergencyReturn() const;
    uint8_t seedsPlanted() const;
    uint8_t seedsRemaining() const;
    bool hasPendingTag() const;
    RfidTag pendingTag() const;

private:
    static constexpr uint8_t MAX_SEEDS = 5;

    MotoronDrive& robot_;

    SafetyState safetyState_;
    MissionState missionState_;
    BaseState baseState_;
    GridState gridState_;
    ExitBaseState exitBaseState_;
    ExploreState exploreState_;
    AlignState alignState_;
    PlantState plantState_;
    ReturnState returnState_;

    bool emergencyReturn_;
    uint32_t stateStartedAt_;
    uint32_t arenaEnteredAt_;
    uint8_t seedsPlanted_;
    bool pendingTagValid_;
    RfidTag pendingTag_;
    mutable char stateNameBuffer_[72];

    void updateNormal();
    void updateBase();
    void updateGrid();
    void updateExitBase();
    void updateExploreGrid();
    void updateAlign();
    void updatePlant();
    void updateReturnHome();
    void updateRevived();

    void transitionSafety(SafetyState nextState);
    void transitionMission(MissionState nextState);
    void transitionBase(BaseState nextState);
    void transitionGrid(GridState nextState);
    void transitionExitBase(ExitBaseState nextState);
    void transitionExplore(ExploreState nextState);
    void transitionAlign(AlignState nextState);
    void transitionPlant(PlantState nextState);
    void transitionReturn(ReturnState nextState);
    void transitionToReturnHome(bool emergencyReturn);

    void enterMissionState();
    void enterBaseState();
    void enterGridState();
    void enterExitBaseState();
    void enterExploreState();
    void enterAlignState();
    void enterPlantState();
    void enterReturnState();
    void reenterCurrentLeafState();
    void stopRobot();
    void clearPendingTag();

    bool stateElapsed(uint32_t durationMs) const;
    bool isInBaseState() const;
    bool isArenaMissionState() const;
    const char* missionStateName(MissionState state) const;
    const char* baseStateName(BaseState state) const;
    const char* gridStateName(GridState state) const;
    const char* exitBaseStateName(ExitBaseState state) const;
    const char* exploreStateName(ExploreState state) const;
    const char* alignStateName(AlignState state) const;
    const char* plantStateName(PlantState state) const;
    const char* returnStateName(ReturnState state) const;
    void logState() const;
};
