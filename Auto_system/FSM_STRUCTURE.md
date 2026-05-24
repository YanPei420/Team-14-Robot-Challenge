# Hierarchical FSM Structure

This document describes the current hierarchical finite-state machine (HFSM). State declarations live in `src/RobotFSM.h`, and implementations are split by layer under `src/fsm/`.

![Term 3 Robot Hierarchical FSM](docs/hfsm_diagram.svg)

## File Layout

```text
src/
|-- RobotFSM.h
`-- fsm/
    |-- Config.h
    |-- Core.cpp
    |-- Names.cpp
    |-- Safety.cpp
    |-- base/
    |   `-- Mission_base.cpp
    `-- grid/
        `-- Mission_grid.cpp
```

| File | Responsibility |
|---|---|
| `src/RobotFSM.h` | State enums, event APIs, query APIs, and private helpers |
| `src/fsm/Core.cpp` | Constructor, `begin()`, top-level `update()`, getters, and shared helpers |
| `src/fsm/Safety.cpp` | Safety layer: `Normal`, `EmergencyStop`, stop and resume handling |
| `src/fsm/base/Mission_base.cpp` | Base mission region: `Idle`, `ExitBase`, `ReturnHome`, `InsideBase` |
| `src/fsm/grid/Mission_grid.cpp` | Grid mission region: exploration, RFID, alignment, planting, stranded/revived |
| `src/fsm/Names.cpp` | State-name formatting and serial state logging |
| `src/fsm/Config.h` | FSM speed, timeout, and action-duration constants |

## Layer Model

```text
SafetyState
|-- Normal
|   `-- MissionState
|       |-- Base
|       |   `-- BaseState + local substates
|       |-- Grid
|       |   `-- GridState + local substates
|       |-- Stranded
|       `-- Revived
`-- EmergencyStop
```

`SafetyState` has the highest priority. When the FSM is in `EmergencyStop`, `update()` returns immediately and does not advance the mission layer.

## Safety Layer

```cpp
enum class SafetyState : uint8_t
{
    Normal,
    EmergencyStop
};
```

| Event | Effect |
|---|---|
| `triggerEmergencyStop()` | Enters `Safety/EmergencyStop` and immediately calls `stop_all()` |
| `clearEmergencyStop()` | Returns to `Normal` and re-enters the current leaf state's action |

Emergency stop does not reset mission/base/grid state. After recovery, `reenterCurrentLeafState()` resumes the previous leaf state.

## Mission Layer

```cpp
enum class MissionState : uint8_t
{
    Base,
    Grid,
    Stranded,
    Revived
};
```

| State | Meaning |
|---|---|
| `Base` | Base-side flow, including exit and return |
| `Grid` | Main arena flow, including RFID, alignment, and planting |
| `Stranded` | Robot is disabled on the field and waits for revive |
| `Revived` | Short pause after revive, then return home |

## Base Region

```cpp
enum class BaseState : uint8_t
{
    Idle,
    ExitBase,
    ReturnHome,
    InsideBase
};
```

```text
Base
|-- Idle
|-- ExitBase
|   |-- RequestClearance
|   |-- LineFollowToDoor
|   |-- WaitForDoor
|   `-- TraverseTunnel
|-- ReturnHome
|   |-- NavigateToAirlock
|   |-- WaitForEntryDoor
|   `-- TraverseTunnel
`-- InsideBase
```

| Event | Transition |
|---|---|
| `startMission()` | `Idle/InsideBase -> ExitBase/RequestClearance` |
| `exitClearanceReceived()` | `RequestClearance -> LineFollowToDoor` |
| `exitDoorDetected()` | `LineFollowToDoor -> WaitForDoor` |
| `exitDoorOpened()` | `WaitForDoor -> TraverseTunnel` |
| `mainArenaReached()` | `Base -> Grid`, and records arena entry time |
| `entryAirlockReached()` | `ReturnHome/NavigateToAirlock -> WaitForEntryDoor` |
| `entryDoorOpened()` | `ReturnHome/WaitForEntryDoor -> TraverseTunnel` |
| `baseReached()` | `ReturnHome -> InsideBase` |

| State | Action |
|---|---|
| `Idle` / `InsideBase` | Stop |
| `ExitBase/RequestClearance` | Stop and wait for clearance |
| `ExitBase/LineFollowToDoor` | `robot.forward(LINE_FOLLOW_SPEED)` |
| `ExitBase/WaitForDoor` | Stop and wait for the door |
| `ExitBase/TraverseTunnel` | `robot.forward(TUNNEL_SPEED)` |
| `ReturnHome/NavigateToAirlock` | `robot.backward(RETURN_SPEED)` |
| `ReturnHome/WaitForEntryDoor` | Stop and wait for the entry door |
| `ReturnHome/TraverseTunnel` | `robot.forward(TUNNEL_SPEED)` |

## Grid Region

```cpp
enum class GridState : uint8_t
{
    ExploreGrid,
    Align,
    Plant
};
```

```text
Grid
|-- ExploreGrid
|   |-- DriveGrid
|   `-- QuerySoilStatus
|-- Align
|   |-- SearchRFID
|   `-- FineAdjustToHole
`-- Plant
    |-- OpenHopper
    |-- DropSeed
    `-- VerifyDrop
```

| Event / Condition | Transition |
|---|---|
| `rfidDetected(coordinate, fertile)` | Valid only in `Grid/ExploreGrid/DriveGrid`; stores the pending tag and enters `QuerySoilStatus` |
| Invalid tag or `SOIL_QUERY_TIMEOUT_MS` elapsed | Clears tag and returns to `DriveGrid` |
| `fertile == true && seedsRemaining() > 0` | `ExploreGrid -> Align` |
| `fertile == false` | Clears tag and returns to `DriveGrid` |
| `SearchRFID` timeout | `SearchRFID -> FineAdjustToHole` |
| `FineAdjustToHole` timeout | `Align -> Plant` |
| `OpenHopper` timeout | `OpenHopper -> DropSeed` |
| `DropSeed` timeout | `DropSeed -> VerifyDrop` |
| `VerifyDrop` timeout | Calls `plantingMechanismDone()` |
| Seed verified and seeds remain | `Plant -> ExploreGrid` |
| No seeds remain | `Plant -> Base/ReturnHome` |

| State | Action |
|---|---|
| `ExploreGrid/DriveGrid` | `robot.forward(GRID_EXPLORE_SPEED)` |
| `ExploreGrid/QuerySoilStatus` | Stop |
| `Align/SearchRFID` | Stop |
| `Align/FineAdjustToHole` | `robot.forward(ALIGN_SPEED)` |
| `Plant/*` | Stop |

## Stranded / Revived

```text
Grid/Base outside safe base area
-> Stranded
-> Revived
-> Base/ReturnHome
```

| Event | Effect |
|---|---|
| `markStranded()` | Enters `Stranded` if the robot is outside the safe base area and safety is normal |
| `reviveFromStranded()` | `Stranded -> Revived` |
| `REVIVE_PAUSE_MS` elapsed | `Revived -> Base/ReturnHome` |

Revived return is a normal return. It does not set `emergencyReturn_`.

## Emergency Return

Emergency return is not a separate `MissionState`. It is a flag on `Base/ReturnHome`:

```cpp
bool emergencyReturn_;
```

It is triggered by:

```cpp
emergencyWarningReceived();
```

Conditions:

- The robot is not in the safe base area.
- The robot is not currently `Stranded`.
- `SafetyState == Normal`.

Serial logs distinguish the two return modes:

```text
Normal/Base/EmergencyReturn/NavigateToAirlock
Normal/Base/ReturnHome/NavigateToAirlock
```

## Top-Level Automatic Rules

`Core.cpp` implements two cross-region rules in `updateNormal()`:

1. If the robot is in `Grid` and `ARENA_TIME_LIMIT_MS` has elapsed since entering the arena, it returns home.
2. If the robot is in `Revived` and `REVIVE_PAUSE_MS` has elapsed, it returns home.

Key constants in `src/fsm/Config.h`:

```cpp
constexpr uint32_t REVIVE_PAUSE_MS = 1000;
constexpr uint32_t ARENA_TIME_LIMIT_MS = 4UL * 60UL * 1000UL;
```

## Typical Flow

Normal mission:

```text
Normal/Base/Idle
-> Normal/Base/ExitBase/RequestClearance
-> Normal/Base/ExitBase/LineFollowToDoor
-> Normal/Base/ExitBase/WaitForDoor
-> Normal/Base/ExitBase/TraverseTunnel
-> Normal/Grid/ExploreGrid/DriveGrid
-> Normal/Grid/ExploreGrid/QuerySoilStatus
-> Normal/Grid/Align/SearchRFID
-> Normal/Grid/Align/FineAdjustToHole
-> Normal/Grid/Plant/OpenHopper
-> Normal/Grid/Plant/DropSeed
-> Normal/Grid/Plant/VerifyDrop
-> Normal/Grid/ExploreGrid/DriveGrid
-> ...
-> Normal/Base/ReturnHome/NavigateToAirlock
-> Normal/Base/ReturnHome/WaitForEntryDoor
-> Normal/Base/ReturnHome/TraverseTunnel
-> Normal/Base/InsideBase
```

Emergency-stop recovery:

```text
Normal/Grid/Plant/DropSeed
-> Safety/EmergencyStop
-> Normal/Grid/Plant/DropSeed
```

## Current main.cpp Note

The current `src/main.cpp` does not instantiate or update `RobotFSM`. It currently runs the WiFi/MQTT heartbeat-gated forward-drive firmware:

```text
heartbeat enable=1 and not timed out -> robot.forward(500)
stop/emergency/disable/enable=0 or heartbeat timeout -> stop_all()
```

So the FSM is currently a compile-ready mission framework, not the active control path in `main.cpp`.
