- [x] Motor Encoder
- [x] IR lib test
- [ ] Motor PD control
- [x] Line Following
- [x] Navigation and map algorithm TODOs in `src/navigation/`
  - [x] Define grid/map coordinate system
  - [x] Map RFID UID to grid coordinate and soil status
  - [x] Choose next target cell during grid exploration
  - [x] Convert navigation decision into `RobotDrive::drive(vx, vy, w)` commands
  - [x] Connect navigator output into `src/fsm/grid/Mission_grid.cpp`
- [ ] M4 motor PD control TODOs in `src/M4/`
  - [ ] Read encoder speed feedback inside M4 motor service
  - [ ] Add per-wheel target speed tracking
  - [ ] Add PD correction before sending Motoron wheel commands
  - [ ] Expose any required tuning/debug data through RPC

## Problem for next week
1.  jump wires for IR are not long enough
2. hopper
