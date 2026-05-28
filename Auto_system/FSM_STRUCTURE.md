# Hierarchical FSM Structure

This document describes the current hierarchical finite-state machine (HFSM).
State declarations live in `src/RobotFSM.h`, and implementations are split by
layer under `src/fsm/`.

![Term 3 Robot Hierarchical FSM](docs/hfsm_diagram.svg)

## File Layout

```text
src/
|-- RobotFSM.h
|-- RobotDrive.h
|-- fsm/
|   |-- FSMconfig.h
|   |-- Core.cpp
|   |-- Names.cpp
|   |-- Safety.cpp
|   |-- FSM_main.cpp
|   |-- FSM_main.h
|   |-- FSM_remote_events.cpp
|   |-- FSM_remote_events.h
|   |-- FSM_auto_events.cpp
|   |-- FSM_auto_events.h
|   |-- base/
|   |   `-- Mission_base.cpp
|   `-- grid/
|       `-- Mission_grid.cpp
`-- navigation/
```

| File | Responsibility |
|---|---|
| `src/RobotFSM.h` | State enums, event APIs, query APIs, navigator hook, and private helpers |
| `src/RobotDrive.h` | Abstract drive interface used by the FSM |
| `src/fsm/Core.cpp` | Constructor, `begin()`, top-level `update()`, getters, and shared helpers |
| `src/fsm/Safety.cpp` | Safety layer: `Normal`, `EmergencyStop`, stop and resume handling |
| `src/fsm/base/Mission_base.cpp` | Base mission region: `Idle`, `ExitBase`, `ReturnHome`, `InsideBase` |
| `src/fsm/grid/Mission_grid.cpp` | Grid mission region: exploration, RFID, alignment, planting, stranded/revived |
| `src/fsm/Names.cpp` | State-name formatting and serial state logging |
| `src/fsm/FSMconfig.h` | FSM speed, timeout, and action-duration constants |
| `src/fsm/FSM_main.cpp` | M7 application wrapper for WiFi, safety, sensors, navigation, and FSM update |
| `src/fsm/FSM_remote_events.*` | MQTT payload parsing into FSM events |
| `src/fsm/FSM_auto_events.*` | Automatic event helpers for doors, tunnels, base arrival, stranded, and RFID lookup |

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

`SafetyState` has the highest priority. When the FSM is in `EmergencyStop`,
`update()` returns immediately and does not advance the mission layer.

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

Emergency stop does not reset mission/base/grid state. After recovery,
`reenterCurrentLeafState()` resumes the previous leaf state.

In the M7 wrapper, the remote safety gate and kill switch drive this layer.
Heartbeat timeout, heartbeat `enable=0`, stop/emergency/disable messages, M4
drive offline, or kill switch activation keep the robot stopped.

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
| `ExitBase/LineFollowToDoor` | `Navigator::driveExitLine()` or `robot.forward(LINE_FOLLOW_SPEED)` |
| `ExitBase/WaitForDoor` | Stop and wait for the door |
| `ExitBase/TraverseTunnel` | `Navigator::driveTunnel()` or `robot.forward(TUNNEL_SPEED)` |
| `ReturnHome/NavigateToAirlock` | `Navigator::driveReturnHome()` or `robot.backward(RETURN_SPEED)` |
| `ReturnHome/WaitForEntryDoor` | Stop and wait for the entry door |
| `ReturnHome/TraverseTunnel` | `Navigator::driveTunnel()` or `robot.forward(TUNNEL_SPEED)` |

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
| `ExploreGrid/DriveGrid` | `Navigator::driveGridExplore()` or `robot.forward(GRID_EXPLORE_SPEED)` |
| `ExploreGrid/QuerySoilStatus` | Stop |
| `Align/SearchRFID` | Stop |
| `Align/FineAdjustToHole` | `Navigator::driveFineAdjust()` or `robot.forward(ALIGN_SPEED)` |
| `Plant/*` | Stop |

When planting finishes, the FSM increments `seedsPlanted_`, clears the pending
tag, and calls `Navigator::markCurrentCellPlanted()` if a navigator is attached.

## Navigation Hook

`RobotFSM` can receive a navigator with:

```cpp
void setNavigator(RobotNavigation::Navigator* navigator);
```

With a navigator attached, the FSM asks navigation to produce movement during
line following, grid exploration, fine adjustment, tunnel traversal, and return
home. Without a navigator, it falls back to fixed-speed `RobotDrive` commands
from `FSMconfig.h`.

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

Emergency return is not a separate `MissionState`. It is a flag on
`Base/ReturnHome`:

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

Key constants in `src/fsm/FSMconfig.h`:

```cpp
constexpr uint32_t REVIVE_PAUSE_MS = 1000;
constexpr uint32_t ARENA_TIME_LIMIT_MS = 4UL * 60UL * 1000UL;
```

## M7 Wrapper Events

`FSM_main.cpp` feeds the FSM from three sources:

- MQTT messages parsed by `FSM_remote_events.*`
- automatic helper functions in `FSM_auto_events.*`
- local sensors: kill switch, revive button, RFID reader, Lidar, IR line sensors, and navigator observations

Recognized MQTT event types include:

```text
start
clearance
exit_door_detected
exit_door_opened
arena_reached
rfid
return
emergency_warning
entry_airlock_reached
entry_door_opened
base_reached
stranded
revive
disable
openAirlockReply
isFertileReply
```

The wrapper sends these server-facing messages:

```text
register
openAirlockA
openAirlockB
isFertile
seedPlanted
```

`FSM_auto_events.cpp` receives the current motion phase, Lidar distance validity,
and line visibility from `FSM_main.cpp`. It uses those inputs to synthesize
clearance, door, tunnel, base, and stranded events.

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

## Current `main.cpp` Note

The current `src/main.cpp` dispatches by core: M7 calls the WiFi/safety/FSM
wrapper in `src/M7/` and `src/fsm/FSM_main.cpp`, while M4 calls the motor
service in `src/M4/`. The full challenge application is therefore the active
firmware entrypoint.
