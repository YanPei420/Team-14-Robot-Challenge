# Auto System Robot - Agent Guide

Target reader: coding agents and developers with no prior project context.

## Project Snapshot

This is an embedded robotics firmware project for the Arduino Giga R1 WiFi. It uses PlatformIO with the Arduino framework and targets the Giga M7 core by default.

Important current state:

- The active `src/main.cpp` is a Motoron drive diagnostic that commands `Robot.right(800)` in a loop.
- The full challenge firmware is in `src/system.cpp` / `src/system.h`.
- `src/main.cpp.bak` contains the thin competition entrypoint that calls `systemSetup()` and `systemLoop()`.
- There is no active `src/RobotFSM.h` or `src/fsm/` tree in the current repository layout.

Do not assume a normal upload runs the autonomous challenge sequence until `src/main.cpp` has been restored to call `systemSetup()` / `systemLoop()`.

## Build Target

Board: Arduino Giga R1 WiFi  
Default environment: `giga_m7`  
Secondary environment: `giga_m4`  
Platform: `ststm32`  
Framework: Arduino  
Language: C++  
Repository root: `Auto_system/`

Common commands:

```bash
pio run
pio run -e giga_m7
pio run -e giga_m4
pio run -e giga_m7 --target upload
pio device monitor -b 115200
```

Managed dependencies:

- `pololu/Motoron`
- `arduino-libraries/Servo`
- `arduino-libraries/ArduinoMqttClient`

## Repository Layout

```text
Auto_system/
|-- platformio.ini
|-- README.md
|-- AGENT.md
|-- include/
|   |-- WiFiHandlerConfig.h
|   `-- Encoder.h
|-- src/
|   |-- main.cpp
|   |-- main.cpp.bak
|   |-- system.h
|   `-- system.cpp
|-- lib/
|   |-- 01 MotoronDrive/
|   |-- 02 KillSwitch/
|   |-- 03 IR/
|   |-- 04 LED/
|   |-- 05 ReviveButton/
|   |-- 06 RFID/
|   |-- 07 Lidar/
|   |-- 08 Servo/
|   |-- 09 LineFollower/
|   |-- 10 WallFollower/
|   |-- 11 Planter/
|   |-- Arduino_MFRC522v2-master/
|   |-- MiniMessenger-main/
|   `-- motoron-arduino-master/
|-- test/
|-- tool/
`-- docs/
```

Generated PlatformIO output lives in `.pio/` and should not be edited by hand.

## Entrypoints

Current active diagnostic in `src/main.cpp`:

```cpp
#include <Arduino.h>
#include "MotoronDrive.h"

MotoronDrive Robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    Robot.begin();
}

void loop()
{
    Robot.right(800);
    Robot.update();
    delay(10);
}
```

Competition entrypoint pattern:

```cpp
#include <Arduino.h>
#include "system.h"

void setup()
{
    systemSetup();
}

void loop()
{
    systemLoop();
}
```

Use the competition pattern when working on `src/system.cpp` behavior.

## Competition System

`src/system.cpp` compiles different code for each Giga core:

- `CORE_CM7`: full autonomous system.
- `CORE_CM4`: minimal serial setup and delay loop.

The M7 system owns these major objects:

- `MotoronDrive robot`
- `IRSensor lineSensors`
- `LidarSensor lidarLeft`, `lidarRight`, `lidarFront`
- `LineFollower lineFollower`
- `WallFollower tunnelFollower`
- `RFIDHandler rfid`
- `KillSwitch killSwitch`
- `ReviveButton reviveButton`
- `LED statusLed`
- `MiniMessenger messenger`
- `Planter planter`

Startup order in `systemSetup()`:

1. Start serial.
2. Initialize sensors, kill switch, revive button, LED, and grid map.
3. Initialize Motoron and encoder speed control.
4. Stop the robot and initialize the planter.
5. Register the MQTT callback.
6. Connect MiniMessenger to WiFi/MQTT.
7. Print readiness and serial help.

Loop order in `systemLoop()`:

1. Process MQTT.
2. Poll serial commands.
3. Send periodic register messages.
4. Update kill switch, revive button, Lidars, and RFID.
5. Apply autonomous/local safety gates.
6. Handle global requests.
7. Update the current state.
8. Update encoder speed control.
9. Update LED and status message.

## Run States

`RunState` currently contains:

```text
Idle
ExitLineToDoor
ExitRequest
ExitWaitDoor
ExitTraverseTunnel
GridDrive
SoilQuery
AlignSearch
FineAdjust
PlantOpen
PlantDrop
PlantVerify
ReturnToAirlock
EntryRequest
EntryWaitDoor
EntryTraverseTunnel
InsideBase
Stranded
ManualControl
PlanterTest
Finished
```

State transitions are centralized through `setState()`. Check that function before adding entry actions, because it already resets control state, sends initial requests, starts planter cycles, and sends mission completion.

## Safety

Autonomous movement requires:

```text
motorReady == true
killSwitch.isSafe() == true
type=heartbeat enable=1 received
heartbeat age <= WIFI_HEARTBEAT_TIMEOUT_MS
```

Manual control and planter test are local-control modes, but the kill switch must still be safe.

When both autonomous and local control are blocked, `systemLoop()`:

```text
manualCommand = Stop
robot.stop_all()
updateStatusLed()
sendStatus()
return
```

Keep new movement paths behind the same safety model unless a test sketch is intentionally isolated.

## MQTT Protocol

Configuration is in `include/WiFiHandlerConfig.h`.

Robot sends:

```text
type=register team_id=14 board_id=Robot14
type=status team_id=14 board_id=Robot14 state=<state> seeds=<n> safety=<0|1>
type=openAirlockB team_id=14 board_id=Robot14
type=openAirlockA team_id=14 board_id=Robot14
type=isFertile team_id=14 board_id=Robot14 tag_id=<uid>
type=seedPlanted team_id=14 board_id=Robot14 tag_id=<uid> count=<n>
type=missionComplete team_id=14 board_id=Robot14 seeds=<n>
```

Robot handles:

```text
type=heartbeat enable=<1|0>
type=start
type=stop
type=emergency
type=disable
type=return
type=emergency_warning
type=stranded
type=revive
type=base_reached
type=openAirlockReply accepted=true airlock=<A|B>
type=isFertileReply tag_id=<uid> x=<n> y=<n> fertile=<true|false> planted=<true|false> blocked=<true|false>
type=rfid tag_id=<uid> x=<n> y=<n> fertile=<true|false> planted=<true|false> blocked=<true|false>
```

RFID and fertility replies can update the internal arena grid. The routing helper currently uses a simple BFS over known, unblocked, line-followable cells.

## Manual Control

Serial commands in the competition system:

```text
S          start autonomous mission
R          request return to base
X          disable remote safety and stop
M          enter manual control
P          leave manual control and stop in Idle
W/B/A/D    manual forward/back/left/right
Q/E        manual rotate left/right
L          manual line follow
G          manual wall follow
O/C/T      run planter 180-degree cycle
0 or Space manual stop
+ / -      manual speed up/down
?          print help
```

In `ManualControl`, `S` maps to backward movement and `W/A/S/D` behaves like keyboard driving.

## Module Notes

`lib/01 MotoronDrive/`

- Wraps front and rear Motoron controllers.
- Supports raw wheel commands, mecanum-style movement helpers, encoder reads, and encoder speed control.
- Direction, addresses, encoder pins, and tuning live in `include/MotorConfig.h`.

`lib/09 LineFollower/`

- Uses IR sensor error and Motoron encoder support.
- Requires encoder speed control by default.
- Used during exit, arena driving, return, and manual line-follow mode.

`lib/10 WallFollower/`

- Uses left/right Lidars to center in tunnel traversal.
- Used for exit and entry tunnel states and manual wall-follow mode.

`lib/11 Planter/`

- Runs an encoder-controlled 180-degree planting cycle.
- `PlanterTest` and `PlantOpen` both rely on `planter.update()` completing or timing out.

## Code Style Guidelines

- Match existing C++ style.
- Use 4-space indentation.
- Opening braces for functions are on the next line.
- Keep hardware constants in config headers.
- Keep production firmware under `src/`.
- Keep manual test sketches under `test/` or `tool/`.
- Avoid editing generated `.pio/` files.
- Before changing motor directions or speed mixing, verify physical wheel orientation.

## Testing

Recommended compile checks:

```bash
pio run -e giga_m7
pio run -e giga_m4
```

For movement changes, test with the robot lifted or physically constrained and keep a safe way to cut motor power.

Useful module sketches live in `test/`, and utility programs live in `tool/`.
