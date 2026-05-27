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

The grid is defined as 4 columns by 4 rows:

```text
A1 B1 C1 D1
A2 B2 C2 D2
A3 B3 C3 D3
A4 B4 C4 D4
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
