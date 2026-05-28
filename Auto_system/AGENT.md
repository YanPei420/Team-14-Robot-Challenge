# Auto System Robot - Agent Guide

Target reader: coding agents with no prior knowledge of this project.

## Project Overview

This is an embedded robotics firmware project for the Arduino Giga R1 WiFi. It uses PlatformIO with the Arduino framework and targets the Giga M7 core by default.

The current `src/main.cpp` firmware is a WiFi/MQTT safety-gated drive program:

```text
valid heartbeat with enable=1 -> drive forward
stop/emergency/disable/enable=0 or heartbeat timeout -> stop all motors
```

The repository also contains a compile-ready hierarchical FSM in `src/RobotFSM.h` and `src/fsm/`, but that FSM is not currently connected to `main.cpp`selects the active firmware
for each Giga core. The current entrypoint calls `M7Core::setup()/loop()` on
`CORE_CM7` and `M4Core::setup()/loop()` on `CORE_CM4`, so the full challenge
application is active by defaul.

## Build Target

Board: Arduino Giga R1 WiFi  
Default environment: `giga_m7`  
Secondary environment: `giga_m4`  
Platform: `ststm32`  
Framework: Arduino  
Language: C++  
Repository root: `Auto_system/`

## Repository Layout

```text
Auto_system/
|-- platformio.ini
|-- AGENT.md
|-- FSM_STRUCTURE.md
|-- navigation_fsm.md
|-- docs/
|   `-- hfsm_diagram.svg
|-- include/
|   `-- WiFiHandlerConfig.h
|-- src/
|   |-- main.cpp
|   |-- RobotFSM.h
|   `-- fsm/
|       |-- Config.h
|       |-- Core.cpp
|       |-- Names.cpp
|       |-- Safety.cpp
|       |-- base/
|       |   `-- Mission_base.cpp
|       `-- grid/
|           `-- Mission_grid.cpp
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
- Default environment: `giga_m7`
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
pio device monitor -b 115200
```

## Current Firmware Flow

`src/main.cpp` creates:

```cpp
MotoronDrive robot;
MiniMessenger messenger;
LED statusLed;
```

Startup:

1. Start `Serial` at `115200`.
2. Initialize `MotoronDrive`.
3. Initialize status LED.
4. Stop all motors.
5. Connect MiniMessenger to WiFi/MQTT using `include/WiFiHandlerConfig.h`.
6. Register `onMessage()` as the incoming-message callback.

Loop:

1. `messenger.loop()` processes WiFi/MQTT work.
2. `sendRegister()` periodically sends `type=register team_id=14 board_id=Robot14`.
3. Connection state changes are printed to serial.
4. Heartbeat timeout disables safety.
5. `movementEnabled()` decides whether motion is allowed.
6. Enabled: LED normal state and `robot.forward(500)`.
7. Disabled: emergency LED state and `robot.stop_all()`.

Motion is allowed only when all conditions are true:

```text
safetyEnabled == true
lastHeartbeatMs != 0
heartbeat age <= WIFI_HEARTBEAT_TIMEOUT_MS
last message is not stop/emergency/disable/enable=0
```

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

Security note: WiFi credentials are currently stored in plain text. Redact them before publishing the repository.

## MotoronDrive

Location: `lib/01 MotoronDrive/`

`MotoronDrive` wraps two `MotoronI2C` controllers on `Wire1`.

Useful API:

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
- `get_wheel_speeds(fl, fr, rl, rr)`

`drive(vx, vy, w)` uses mecanum-style mixing:

```text
frontLeft  = vx - vy - w
frontRight = vx + vy + w
rearLeft   = vx + vy - w
rearRight  = vx - vy + w
```

All wheel commands are clamped by `MOTOR_MAX_SPEED`.

## LED

Location: `lib/04 LED/`

Current `main.cpp` uses:

- `showNormal()`
- `showEmergency()`

Check `LEDConfig.h` for pin assignments.

## Hierarchical FSM

The FSM is declared in `src/RobotFSM.h` and documented in `FSM_STRUCTURE.md`.

Implementation layout:

```text
src/fsm/
|-- Core.cpp
|-- Names.cpp
|-- Safety.cpp
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

Important note: the FSM currently compiles, but `main.cpp` does not instantiate or update it.

## Code Style Guidelines

- Match the existing C++ style.
- Use 4-space indentation.
- Opening braces for functions are on the next line.
- Keep hardware constants in config headers.
- Keep production firmware under `src/`.
- Keep manual test sketches under `test/` or `tool/`.
- Avoid editing generated `.pio/` files.
- Before changing motor directions or speed mixing, verify physical wheel orientation.

## Testing

Recommended checks:

```bash
pio run -e giga_m7
pio run -e giga_m4
```

For movement changes, test with the robot lifted or physically constrained and keep a safe way to cut motor power.

## Operational Notes

- `stop_all()` should be called during startup and before unsafe states.
- `MiniMessenger` heartbeat loss must fail safe to stopped motors.
- The current firmware does not use `RobotFSM` yet, so FSM changes should be validated by build and by a future integration test when connected to `main.cpp`.
