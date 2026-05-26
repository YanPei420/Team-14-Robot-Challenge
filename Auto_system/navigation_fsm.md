# Term 3 Challenge Hierarchical FSM

This document is the short reference for the current hierarchical FSM. The detailed explanation is in `FSM_STRUCTURE.md`.

The FSM is declared in:

```text
src/RobotFSM.h
```

The implementation is split by layer:

```text
src/fsm/
|-- Config.h
|-- Core.cpp
|-- Names.cpp
|-- Safety.cpp
|-- base/
|   `-- Mission_base.cpp
`-- grid/
    `-- Mission_grid.cpp
```

![Term 3 Robot Hierarchical FSM](docs/hfsm_diagram.svg)

## State Tree

```text
Robot
|-- SafetyState
|   |-- Normal
|   `-- EmergencyStop
|
`-- MissionState
    |-- Base
    |   |-- Idle
    |   |-- ExitBase
    |   |   |-- RequestClearance
    |   |   |-- LineFollowToDoor
    |   |   |-- WaitForDoor
    |   |   `-- TraverseTunnel
    |   |-- ReturnHome
    |   |   |-- NavigateToAirlock
    |   |   |-- WaitForEntryDoor
    |   |   `-- TraverseTunnel
    |   `-- InsideBase
    |
    |-- Grid
    |   |-- ExploreGrid
    |   |   |-- DriveGrid
    |   |   `-- QuerySoilStatus
    |   |-- Align
    |   |   |-- SearchRFID
    |   |   `-- FineAdjustToHole
    |   `-- Plant
    |       |-- OpenHopper
    |       |-- DropSeed
    |       `-- VerifyDrop
    |
    |-- Stranded
    `-- Revived
```

## Mermaid Diagram

```mermaid
stateDiagram-v2
    [*] --> Normal
    Normal --> EmergencyStop: triggerEmergencyStop()
    EmergencyStop --> Normal: clearEmergencyStop()

    state Normal {
        [*] --> Base

        state Base {
            [*] --> Idle
            Idle --> ExitBase: startMission()
            InsideBase --> ExitBase: startMission()

            state ExitBase {
                [*] --> RequestClearance
                RequestClearance --> LineFollowToDoor: exitClearanceReceived()
                LineFollowToDoor --> WaitForDoor: exitDoorDetected()
                WaitForDoor --> TraverseTunnel: exitDoorOpened()
            }

            state ReturnHome {
                [*] --> NavigateToAirlock
                NavigateToAirlock --> WaitForEntryDoor: entryAirlockReached()
                WaitForEntryDoor --> TraverseTunnel: entryDoorOpened()
            }

            ReturnHome --> InsideBase: baseReached()
        }

        Base --> Grid: mainArenaReached()

        state Grid {
            [*] --> ExploreGrid

            state ExploreGrid {
                [*] --> DriveGrid
                DriveGrid --> QuerySoilStatus: rfidDetected()
                QuerySoilStatus --> DriveGrid: infertile or timeout
            }

            ExploreGrid --> Align: fertile && seedsRemaining > 0

            state Align {
                [*] --> SearchRFID
                SearchRFID --> FineAdjustToHole: search timeout
            }

            Align --> Plant: fine adjust complete

            state Plant {
                [*] --> OpenHopper
                OpenHopper --> DropSeed: open timeout
                DropSeed --> VerifyDrop: drop timeout
            }

            Plant --> ExploreGrid: seed verified && seeds remain
        }

        Grid --> Base: return home or emergency warning
        Grid --> Stranded: markStranded()
        Base --> Stranded: markStranded()
        Stranded --> Revived: reviveFromStranded()
        Revived --> Base: revive pause elapsed
    }
```

## Why This Split

The software now mirrors the physical challenge layout:

| Layer | Meaning |
|---|---|
| `SafetyState` | Highest-priority emergency stop layer |
| `MissionState::Base` | Start area, exit tunnel, return tunnel, and final base state |
| `MissionState::Grid` | RFID planting arena |
| Local substates | Small action phases inside Base or Grid |

## Serial Test Events

These events are the intended FSM test inputs if `RobotFSM` is connected from `main.cpp`:

```text
s  start mission
c  exit clearance received
d  exit door detected
o  door opened, used for both exit and entry depending on current state
a  main arena reached
f  fertile RFID tag detected
i  infertile RFID tag detected
r  emergency return warning
e  entry airlock reached
b  base reached
x  mark stranded
v  revive from stranded
```

Example:

```text
s c d o a f e o b
```

## Current Firmware Note

The current `src/main.cpp` does not call `RobotFSM` yet. It currently runs the WiFi/MQTT heartbeat-gated forward-drive program.
