# Navigation and Map

This folder contains high-level navigation and map algorithms.

Keep this logic on M7. The navigator should decide where the robot should go
next, while M4 stays responsible for low-level motor and encoder control.

## Files

- `GridMap.h/.cpp`: grid coordinates, visited/fertile cells, planted cells, and RFID lookup
- `Navigator.h/.cpp`: target selection, line-following motion, obstacle stop gate, and drive commands
- `PoseEstimator.h/.cpp`: current grid position and selected target estimate
- `NavigationRuntime.h/.cpp`: shared M7 navigator singleton used by FSM glue code

## Coordinate System

The grid is defined as 9 columns by 9 rows:

```text
A1 B1 C1 D1 E1 F1 G1 H1 I1
A2 B2 C2 D2 E2 F2 G2 H2 I2
A3 B3 C3 D3 E3 F3 G3 H3 I3
A4 B4 C4 D4 E4 F4 G4 H4 I4
A5 B5 C5 D5 E5 F5 G5 H5 I5
A6 B6 C6 D6 E6 F6 G6 H6 I6
A7 B7 C7 D7 E7 F7 G7 H7 I7
A8 B8 C8 D8 E8 F8 G8 H8 I8
A9 B9 C9 D9 E9 F9 G9 H9 I9
```

Coordinates are stored as zero-based column/row pairs internally and formatted
as `A1` style text at FSM/RFID boundaries.

## Integration

- `FSM_auto_events.cpp` delegates local RFID UID lookup to `Navigator`.
- `FSM_main.cpp` feeds IR line readings into `Navigator` once per loop.
- `RobotFSM` keeps mission states and asks `Navigator` for motion commands while
  driving exit lines, exploring the grid, fine adjusting, crossing tunnels, and
  returning home.
- Direct motor control still goes through the `RobotDrive` interface.

RFID UID data is configured at runtime with `GridMap::setRfidTag()`. Remote MQTT
RFID events that already include `coordinate=` and `fertile=` are accepted
directly and update the navigator map.
