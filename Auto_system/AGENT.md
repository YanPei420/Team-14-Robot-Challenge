# Auto System Robot - Agent Guide

Target reader: AI coding agents with no prior knowledge of this project.

## Project Overview

This is an embedded robotics firmware project for the Arduino Giga R1 WiFi. It controls a mobile robot chassis through two Pololu Motoron I2C motor controllers, includes support modules for a hardware kill switch, UDP WiFi stop commands, and I2C distance sensing, and is built with PlatformIO using the Arduino framework.

The current `src/main.cpp` firmware is a simple forward speed test: it initializes the motor drive and repeatedly runs the robot forward at speed `100`, stops, then runs forward at speed `500`.

Board: Arduino Giga R1 WiFi
Default core: M7 (`giga_r1_m7`)
Secondary environment: M4 (`giga_r1_m4`)
Platform: `ststm32`
Framework: Arduino
Language: C++
Repository root: `Auto_system/`

## Repository Layout

```text
Auto_system/
|-- platformio.ini              # PlatformIO build configuration
|-- AGENT.md                    # AI-agent project guide
|-- navigation_fsm.md           # Planned/previous navigation FSM documentation
|-- src/
|   `-- main.cpp                # Current firmware entry point
|-- lib/                        # PlatformIO local libraries
|   |-- 01 MotoronDrive/        # Motoron-based chassis drive wrapper
|   |-- 02 KillSwitch/          # Emergency switch helper
|   |-- 03 WiFiDrive/           # WiFi + UDP stop-command helper
|   |-- 04 LED/                 # Red/green status LED helper
|   |-- 05 ReviveButton/        # Revive button helper
|   |-- DistanceSensor/         # I2C distance sensor helper
|   |-- IR/                     # Placeholder local library metadata
|   |-- Lidar/                  # Placeholder local library metadata
|   `-- RFID/                   # Placeholder local library metadata
|-- test/                       # Manual/experimental sketches
|   |-- distance_sensor.cpp
|   |-- KillSwitchs_LED_Button.cpp
|   |-- turning.cpp
|   `-- U-turn.cpp
`-- .vscode/                    # VS Code / PlatformIO workspace files
```

Generated PlatformIO output lives in `.pio/` and should not be edited by hand.

## Build System

### Key Configuration (`platformio.ini`)

Default environment:

```ini
default_envs = giga_m7
```

Common settings:

- Platform: `ststm32`
- Framework: `arduino`
- Monitor speed: `115200`
- Managed dependencies:
  - `pololu/Motoron @ ^1.3.0`
  - `arduino-libraries/Servo @ ^1.2.1`

Environments:

- `giga_m7` - main target for the Arduino Giga R1 M7 core.
  - Board: `giga_r1_m7`
  - Build flags: `-D CORE_CM7`, `-Iinclude`, `-Wl,--allow-multiple-definition`
- `giga_m4` - secondary target for the Arduino Giga R1 M4 core.
  - Board: `giga_r1_m4`
  - Build flags: `-D CORE_CM4`

### Common Commands

```bash
# Build the default M7 firmware
pio run

# Build the M7 firmware explicitly
pio run -e giga_m7

# Build the M4 firmware
pio run -e giga_m4

# Upload to the M7 core
pio run -e giga_m7 --target upload

# Open serial monitor at 115200 baud
pio device monitor -b 115200
```

Use the VS Code PlatformIO extension if working interactively.

## Hardware Architecture

### Board

Target hardware is the Arduino Giga R1 WiFi. The main firmware is currently configured for the M7 core.

### Serial

`Serial` is used as the primary debug console at `115200` baud. `setup()` waits for `Serial` before initializing the motor driver.

### I2C

MotoronDrive uses `Wire1` for the Pololu Motoron motor controllers. `MotoronDrive.h` defines `Wire` as `Wire1` before including `Motoron.h`, and `MotoronDrive::begin()` calls `Wire1.begin()`.

The distance sensor module accepts any `TwoWire&` bus in its constructor.

### Motoron Addresses and Motor Channels

Defined in `lib/01 MotoronDrive/include/MotorConfig.h`:

| Device | Address |
| --- | --- |
| Front Motoron board | `16` (`0x10`) |
| Rear Motoron board | `17` (`0x11`) |

Motor channel mapping:

| Motor | Board | Channel |
| --- | --- | --- |
| Front left | Front board | `1` |
| Front right | Front board | `2` |
| Rear left | Rear board | `1` |
| Rear right | Rear board | `2` |

`MOTOR_MAX_SPEED` is `800`. All `MotoronDrive` speed commands are clamped to `[-800, 800]`.

### Pins

Defined in `lib/04 LED/include/LEDConfig.h`:

| Function | Pin |
| --- | --- |
| Green LED | `34` |
| Red LED | `32` |

Defined in `lib/05 ReviveButton/include/ReviveButtonConfig.h`:

| Function | Pin / Value |
| --- | --- |
| Revive button pin | `24` |
| Button active state | `LOW` |

Defined in `lib/02 KillSwitch/include/KillSwitchConfig.h`:

| Function | Pin / Value |
| --- | --- |
| Kill switch pin | `22` |
| Emergency active state | `HIGH` |
| Normal state | `LOW` |

`KillSwitch::begin()` configures the switch pin as `INPUT_PULLUP`.

## Module Reference

### `MotoronDrive`

Location: `lib/01 MotoronDrive/`

Wraps two `MotoronI2C` instances: one for the front controller and one for the rear controller.
The module is split into three parts:

- `MotoronDrive` - low-level Motoron initialization, speed limiting, four-wheel output, and compatibility movement wrappers.
- `MotoronChassis` - high-level chassis movement helpers such as forward, strafe, rotate, and `drive(vx, vy, w)`.
- `MotoronMath` - mecanum inverse-kinematics math and proportional wheel-speed scaling.

Public API:

- `begin()` - starts `Wire1`, reinitializes both Motoron boards, disables CRC, clears reset flags, and stops all motors.
- `set_front_left(speed)`
- `set_front_right(speed)`
- `set_rear_left(speed)`
- `set_rear_right(speed)`
- `set_all(fl, fr, rl, rr)`
- `get_wheel_speeds(fl, fr, rl, rr)` - reads back the last commanded wheel speeds.
- `drive(vx, vy, w)` - mecanum chassis inverse kinematics, where `vx` is forward/back, `vy` is right/left, and `w` is rotation.
- `set_max_speed(maxSpeed)`
- `get_max_speed()`
- `forward(speed)`
- `backward(speed)`
- `left(speed)`
- `right(speed)`
- `rotate_left(speed)`
- `rotate_right(speed)`
- `stop()` - alias for `stop_all()`.
- `stop_all()`

Notes:

- `drive()` uses X-configuration mecanum inverse kinematics:
  - `fl = vx - vy - w`
  - `fr = vx + vy + w`
  - `rl = vx + vy - w`
  - `rr = vx - vy + w`
- If any computed wheel speed exceeds the configured max speed, all four wheel speeds are scaled down proportionally.
- `forward()` and `backward()` are wrappers around `drive(vx, 0, 0)`.
- `left()` and `right()` are wrappers around `drive(0, vy, 0)`.
- `rotate_left()` and `rotate_right()` are wrappers around `drive(0, 0, w)`.

### `KillSwitch`

Location: `lib/02 KillSwitch/`

Small helper for reading a digital emergency switch.

Public API:

- `begin()` - configures the switch pin.
- `update()` - reads the pin and stores whether the switch is triggered.
- `isTriggered()` - returns the stored emergency state.
- `isSafe()` - returns `true` when the switch is not triggered.

Configuration is in `KillSwitchConfig.h`.

### `WiFiHandler`

Location: `lib/03 WiFiDrive/`

Connects to WiFi and listens for UDP packets. If it receives the configured stop command, it marks `stopTriggered = true`.

Configuration in `WiFiHandlerConfig.h`:

| Setting | Value |
| --- | --- |
| SSID | `"Xiao Mi 15 Ultra"` |
| Password | `"00000000"` |
| UDP port | `4210` |
| UDP buffer size | `255` |
| Stop command | `"Stop"` |

Public API:

- `begin()` - connects to WiFi, prints the IP address, and starts UDP listening.
- `update()` - polls UDP and checks for the stop command.
- `isStopTriggered()` - returns whether the stop command has been received.
- `getIP()` - returns `WiFi.localIP()`.

Security note: WiFi credentials are currently committed in plain text in `WiFiHandlerConfig.h`. Redact or replace them before publishing the repository.

### `DistanceSensor`

Location: `lib/DistanceSensor/`

Reads distance data over I2C.

Public API:

- `DistanceSensor(TwoWire& wireBus, uint8_t sensorAddress)`
- `begin()` - starts the chosen I2C bus and waits 500 ms.
- `readDistanceCM()` - reads registers `0x5E` and `0x5F`, converts the raw value to centimeters, and returns `-1.0f` if two bytes are not available.

### Placeholder Libraries

The following directories currently contain only `library.json` metadata:

- `lib/IR/`
- `lib/Lidar/`
- `lib/RFID/`

Add `include/` and `src/` files before treating these as implemented modules.

### Navigation FSM Notes

`navigation_fsm.md` documents an intended finite-state-machine design with `Idle`, `LineFollow`, and `WallFollow` states, plus RFID events. The implementation files referenced there, such as `src/tasks/navigation/navigation_fsm.cpp`, are not present in the current repository. Treat this file as planning/reference documentation unless those sources are added later.

## Current Firmware Flow

`src/main.cpp`:

1. Includes Arduino, `MotorConfig.h`, and `MotoronDrive.h`.
2. Creates a global `MotoronDrive Robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR)`.
3. In `setup()`:
   - Starts `Serial` at `115200`.
   - Waits for serial connection.
   - Calls `Robot.begin()`.
   - Prints `FORWARD SPEED TEST`.
4. In `loop()`:
   - Prints `FORWARD 100`.
   - Drives forward at speed `100` for 3 seconds.
   - Stops for 1 second.
   - Prints `FORWARD 500`.
   - Drives forward at speed `500` for 3 seconds.
   - Stops for 2 seconds.

## Code Style Guidelines

There is no `.clang-format` file in this repository at the moment. Match the existing style unless the team adds a formatter.

Existing style:

- C++ headers use either `#pragma once` or include guards.
- Indentation is generally 4 spaces.
- Opening braces for functions are on the next line.
- Many function calls are split across multiple lines in the local libraries.
- Constants are currently defined with `#define` in config headers.
- Debug output uses `Serial.print()` and `Serial.println()`.

When adding new code:

- Keep hardware constants in the relevant config header.
- Prefer clear module boundaries under `lib/<ModuleName>/include` and `lib/<ModuleName>/src`.
- Avoid editing generated `.pio/` dependency files.
- Keep manual test sketches in `test/` separate from production firmware in `src/`.
- Be careful with motor directions; verify physical wheel orientation before changing movement helpers.

## Testing & Quality

There are no automated unit tests configured in PlatformIO yet.

The `test/` directory contains manual/experimental sketches:

- `distance_sensor.cpp`
- `KillSwitchs_LED_Button.cpp`
- `turning.cpp`
- `U-turn.cpp`

Recommended checks:

```bash
pio run -e giga_m7
pio run -e giga_m4
```

For hardware changes, verify on the robot with the serial monitor open and a safe way to cut motor power.

## Security & Operational Notes

- WiFi SSID and password are stored in plain text in `lib/03 WiFiDrive/include/WiFiHandlerConfig.h`.
- Motoron CRC is disabled in `MotoronDrive::begin()` via `disableCrc()`.
- `stop_all()` should be called before and after motor tests.
- Keep the robot lifted or physically constrained when testing new movement code.
- The kill switch helper exists, but the current `src/main.cpp` speed test does not use it.
- UDP stop support exists in `WiFiHandler`, but the current `src/main.cpp` speed test does not use it.
