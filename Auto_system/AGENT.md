# Auto System Robot - Agent Guide

Target reader: coding agents with no prior knowledge of this project.

## Project Overview

This is an embedded robotics firmware project for the Arduino Giga R1 WiFi.
It uses PlatformIO with the Arduino framework and builds both Giga cores by
default:

- `giga_m7` / `CORE_CM7`: high-level application, WiFi/MQTT, sensors, FSM, and RPC client
- `giga_m4` / `CORE_CM4`: low-level Motoron chassis service, encoder setup, and RPC endpoints

Important current-entrypoint note: `src/main.cpp` is currently a standalone
TF-Luna Lidar UART test sketch. The fuller challenge application code exists in
`src/M7/`, `src/M4/`, and `src/fsm/FSM_main.cpp`, but `main.cpp` does not call
`M7Core::setup()/loop()` or `M4Core::setup()/loop()` at the moment.

The repository also contains a compile-ready hierarchical FSM in
`src/RobotFSM.h` and `src/fsm/`. The FSM is integrated with the M7 challenge
wrapper in `src/fsm/FSM_main.cpp`, uses `RobotDrive`, and can delegate motion
decisions to `src/navigation/`.

## Build Target

- Board: Arduino Giga R1 WiFi
- Default environments: `giga_m7`, `giga_m4`
- Platform: `ststm32`
- Framework: Arduino
- Language: C++
- Repository root: `Auto_system/`

## Repository Layout

```text
Auto_system/
|-- platformio.ini
|-- AGENT.md
|-- FSM_STRUCTURE.md
|-- navigation_fsm.md
|-- ToDoList.md
|-- docs/
|   `-- hfsm_diagram.svg
|-- include/
|   `-- WiFiHandlerConfig.h
|-- src/
|   |-- main.cpp
|   |-- RobotDrive.h
|   |-- RobotFSM.h
|   |-- M7/
|   |-- M4/
|   |-- fsm/
|   |-- navigation/
|   `-- main.cpp.bac
|-- lib/
|   |-- 01 MotoronDrive/
|   |-- 02 KillSwitch/
|   |-- 03 IR/
|   |-- 04 LED/
|   |-- 05 ReviveButton/
|   |-- 06 RFID/
|   |-- 07 Lidar/
|   |-- Arduino_MFRC522v2-master/
|   |-- MiniMessenger-main/
|   `-- motoron-arduino-master/
|-- test/
`-- tool/
```

Generated PlatformIO output lives in `.pio/` and should not be edited by hand.

## Build System

Key configuration is in `platformio.ini`.

Common settings:

- Platform: `ststm32`
- Framework: `arduino`
- Monitor speed: `115200`
- Default environments: `giga_m7`, `giga_m4`
- Managed dependencies:
  - `pololu/Motoron`
  - `arduino-libraries/Servo`
  - `arduino-libraries/ArduinoMqttClient`

Common commands:

```bash
pio run
pio run -e giga_m7
pio run -e giga_m4
pio run -e giga_m7 --target upload
pio run -e giga_m4 --target upload
pio device monitor -b 115200
```

## Current `main.cpp`

`src/main.cpp` currently creates a `LidarSensor` and prints TF-Luna distance
readings:

```text
Serial 115200
LIDAR_SERIAL begin through LidarSensor
loop: if lidar.update(), print getDistanceCM()
```

This is useful for sensor bring-up, but it is not the full robot application.

## Challenge Application Modules

The challenge application is split into core-specific modules.

M7 side:

- `src/M7/M7_main.cpp`: starts Serial/RPC and calls `RobotApp::fsmSetup()` and `RobotApp::fsmLoop()`
- `src/M7/M7DriveProxy.*`: sends drive commands to M4 through RPC
- `src/fsm/FSM_main.cpp`: WiFi/MQTT, heartbeat safety gate, kill switch, LED, revive button, RFID, Lidar, IR line observations, and FSM update loop
- `src/fsm/FSM_remote_events.*`: parses MQTT payloads into FSM events
- `src/fsm/FSM_auto_events.*`: automatic event helpers for doors, tunnels, base arrival, stranded, and RFID UID lookup

M4 side:

- `src/M4/M4_main.cpp`: calls the M4 motor service
- `src/M4/M4MotorService.*`: owns `MotoronDrive`, sets up encoders, exposes RPC endpoints, and periodically refreshes motor commands

The active Arduino `setup()`/`loop()` still live in `src/main.cpp`. To run the
challenge wrapper instead of the Lidar test, wire `main.cpp` to the M7/M4 module
entrypoints for the selected core.

## WiFi / MQTT Configuration

Configuration is in `include/WiFiHandlerConfig.h`:

| Setting | Purpose |
|---|---|
| `WIFI_SSID` / `WIFI_PASSWORD` | Network credentials |
| `BROKER_HOST` / `BROKER_PORT` | MQTT broker |
| `GROUP_ID` | Team id |
| `BOARD_ID` | This robot board id |
| `SERVER_BOARD_ID` | Server board id |
| `WIFI_REGISTER_INTERVAL_MS` | Register-message interval |
| `WIFI_HEARTBEAT_TIMEOUT_MS` | Heartbeat timeout before stopping |

Security note: WiFi credentials are currently stored in plain text. Redact them
before publishing the repository.

## Remote Safety Gate

`FSM_main.cpp` keeps motion disabled unless all of these are true:

```text
M4 drive proxy is ready
latest heartbeat has enable=1
heartbeat age <= WIFI_HEARTBEAT_TIMEOUT_MS
kill switch is not triggered
```

If the gate closes, the M7 wrapper triggers `RobotFSM::triggerEmergencyStop()`,
shows the emergency LED state, and skips normal FSM updates for that loop.

Recognized MQTT downlink event payloads include:

```text
type=start
type=clearance
type=exit_door_detected
type=exit_door_opened
type=arena_reached
type=rfid coordinate=A1 fertile=1
type=return
type=emergency_warning
type=entry_airlock_reached
type=entry_door_opened
type=base_reached
type=stranded
type=revive
type=disable
type=openAirlockReply accepted=1
type=isFertileReply x=1 y=1 fertile=1 planted=0 tag_id=...
```

The M7 wrapper also sends server messages:

```text
type=register team_id=... board_id=...
type=openAirlockA team_id=... board_id=...
type=openAirlockB team_id=... board_id=...
type=isFertile team_id=... board_id=... tag_id=...
type=seedPlanted team_id=... board_id=... tag_id=...
```

## Motor Control

`RobotFSM` only talks to the abstract `RobotDrive` interface in
`src/RobotDrive.h`.

On M7, `M7DriveProxy` implements `RobotDrive` by sending RPC commands:

- `m4_motor_begin`
- `m4_motor_drive`
- `m4_motor_set_all`
- `m4_motor_stop_all`
- `m4_motor_encoders_ready`

On M4, `M4MotorService` owns the real `MotoronDrive` and refreshes the last
drive command every `50 ms` while motion is active.

`MotoronDrive` lives in `lib/01 MotoronDrive/` and provides:

- `begin()`
- `forward(speed)`
- `backward(speed)`
- `left(speed)`
- `right(speed)`
- `rotate_left(speed)`
- `rotate_right(speed)`
- `drive(vx, vy, w)`
- `stop_all()`
- `set_all(fl, fr, rl, rr)`
- encoder helpers through `MotoronDriveEncoders`

Before changing motor directions or speed mixing, verify physical wheel
orientation.

## Navigation

High-level navigation lives in `src/navigation/`.

- `GridMap.*`: grid coordinates, visited/fertile/planted cells, RFID lookup
- `Navigator.*`: target selection, line following, obstacle stop gate, and drive commands
- `PoseEstimator.*`: current grid position and selected target estimate
- `NavigationRuntime.*`: shared M7 navigator singleton

`RobotFSM::setNavigator()` connects the FSM to `Navigator`. With a navigator
attached, base, grid, align, tunnel, and return states ask navigation for motion
commands. Without one, the FSM falls back to simple fixed-speed motor commands.

## Hierarchical FSM

The FSM is declared in `src/RobotFSM.h` and documented in `FSM_STRUCTURE.md`.

Implementation layout:

```text
src/fsm/
|-- FSMconfig.h
|-- Core.cpp
|-- Names.cpp
|-- Safety.cpp
|-- FSM_main.cpp
|-- FSM_main.h
|-- FSM_remote_events.cpp
|-- FSM_remote_events.h
|-- FSM_auto_events.cpp
|-- FSM_auto_events.h
|-- base/
|   `-- Mission_base.cpp
`-- grid/
    `-- Mission_grid.cpp
```

High-level state tree:

```text
SafetyState
|-- Normal
|   `-- MissionState
|       |-- Base
|       |-- Grid
|       |-- Stranded
|       `-- Revived
`-- EmergencyStop
```

## Code Style Guidelines

- Match the existing C++ style.
- Use 4-space indentation.
- Opening braces for functions are on the next line in most project C++ files.
- Keep hardware constants in config headers.
- Keep production firmware under `src/`.
- Keep manual test sketches under `test/` or `tool/`.
- Avoid editing generated `.pio/` files.
- Keep direct motor controller refresh and encoder handling on M4.
- Keep WiFi, MQTT, mission policy, and navigation decisions on M7.

## Testing

Recommended build checks:

```bash
pio run -e giga_m7
pio run -e giga_m4
```

For movement changes, test with the robot lifted or physically constrained and
keep a safe way to cut motor power.

## Operational Notes

- `stop_all()` should be called during startup and before unsafe states.
- Heartbeat loss must fail safe to stopped motors.
- `FSM_auto_events.cpp` still contains calibration thresholds for Lidar door
  detection, tunnel timing, base arrival, stranded detection, and RFID lookup.
- M4 motor PD control is still a TODO; the service currently stores target
  speeds and refreshes commands, but does not close the loop with encoder speed.
