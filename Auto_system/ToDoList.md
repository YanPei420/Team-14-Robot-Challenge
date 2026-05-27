# To Do List

- [x] Motor Encoder
- [x] IR lib test
- [x] Line Following
- [x] Navigation and map algorithm TODOs in `src/navigation/`
  - [x] Define grid/map coordinate system
  - [x] Map RFID UID to grid coordinate and soil status
  - [x] Choose next target cell during grid exploration
  - [x] Convert navigation decision into `RobotDrive::drive(vx, vy, w)` commands
  - [x] Connect navigator output into `src/fsm/grid/Mission_grid.cpp`
- [x] Split challenge code into M7/M4 modules
  - [x] M7 wrapper for WiFi/MQTT, safety, sensors, FSM, and navigation
  - [x] M4 RPC motor service with Motoron command refresh
  - [x] M7 `RobotDrive` proxy over RPC
- [ ] Select final firmware entrypoint in `src/main.cpp`
  - [ ] Keep Lidar test only while calibrating TF-Luna
  - [ ] Switch to M7/M4 wrapper before full challenge run
- [ ] Calibrate automatic FSM event detection in `src/fsm/FSM_auto_events.cpp`
  - [ ] Door-near Lidar threshold
  - [ ] Door-open Lidar threshold
  - [ ] Tunnel traversal timing
  - [ ] Base arrival line confirmation
  - [ ] Stranded detection signal
- [ ] Finalize server protocol calibration
  - [ ] Confirm `openAirlockA/B` request and `openAirlockReply` fields
  - [ ] Confirm `isFertile` / `isFertileReply` fields
  - [ ] Confirm `seedPlanted` payload fields
  - [ ] Fill RFID UID map data for local fallback
- [ ] M4 motor PD control TODOs in `src/M4/`
  - [ ] Read encoder speed feedback inside M4 motor service
  - [ ] Add per-wheel target speed tracking
  - [ ] Add PD correction before sending Motoron wheel commands
  - [ ] Expose any required tuning/debug data through RPC

## Problem for next week

1. Jump wires for IR are not long enough
2. Hopper
