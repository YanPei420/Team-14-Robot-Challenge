# Testing And Calibration Evidence

This document collects the evidence that supports the viva/test run. It is written as a short engineering log so the team can explain what was tested, what worked, what still needs care, and which code or constants were involved.

## Build And Upload Evidence

| Check | Evidence | Result |
| --- | --- | --- |
| PlatformIO project builds for the main core | `pio run -e giga_m7` | Use before every upload. |
| Final entrypoint is identifiable | `src/main.cpp` calls `systemSetup()` and `systemLoop()` | Uploading the default `giga_m7` environment runs the autonomous system. |
| Serial monitor configured | `platformio.ini` sets `monitor_speed = 115200` | Serial commands and debug logs use 115200 baud. |
| M4 core guarded | `src/system.cpp` has a `CORE_CM4` stub branch | The main challenge logic is intentionally on the M7 core. |

## Module Test Sketches

The `test/` folder contains focused sketches used to isolate behaviours before integrating them into `src/system.cpp`.

| Test sketch | Purpose | What to look for | Related production code |
| --- | --- | --- | --- |
| `test/Motor_run.cpp` | Basic Motoron output check | Wheels respond in the expected direction; `stop_all()` stops motion. | `lib/01 MotoronDrive/` |
| `test/encoder_test.cpp` | Encoder count and direction check | Counts increase consistently for each wheel and match configured direction signs. | `MotorEncoder`, `MotorConfig.h` |
| `test/line_follow.cpp` | IR line following check | Robot follows dark line, searches when line is lost, and logs line error/turn values. | `LineFollower` |
| `test/wall_follow.cpp` | Tunnel centering check | Left/right Lidar distances remain fresh and steering keeps robot between walls. | `WallFollower`, `LidarSensor` |
| `test/Lidar_test.cpp` | TF-Luna UART frame check | Distance values are stable and invalid readings are rejected. | `LidarSensor` |
| `test/RFID_test.cpp` | RFID UID read check | Soil tag UID is printed and repeated reads are debounced in integration. | `RFIDHandler` |
| `test/servo_test.cpp` | Servo/actuator sanity check | Servo positions move to expected limits without binding. | `ServoSweep` |
| `test/obstacle_detection.cpp` | Front obstacle threshold check | Robot can detect objects inside the configured obstacle distance. | `RunState::GridDrive` |
| `test/grid_nav.cpp` | Grid route decision check | Routeable cells are limited to the line-following rows. | grid helpers in `system.cpp` |
| `test/wifi.cpp`, `test/trial_run_wifi.cpp` | MQTT and server message check | Registration, heartbeat, status, airlock, soil, and mission messages use the expected fields. | `MiniMessenger`, `WiFiHandlerConfig.h` |

## Calibration Values Used In The Final Firmware

| Area | Value | File | Reason |
| --- | --- | --- | --- |
| Motor max command | `MOTOR_MAX_SPEED = 800` | `lib/01 MotoronDrive/include/MotorConfig.h` | Keeps commands inside the Motoron speed range used by module tests. |
| Motor acceleration/deceleration | `300` / `600` | `MotorConfig.h` | Reduces sudden starts while allowing faster stops. |
| Encoder speed interval | `50 ms` | `MotorConfig.h` | Gives stable wheel speed corrections without excessive command traffic. |
| IR sensor count | `9` | `lib/03 IR/include/IRConfig.h` | Matches the QTR-HD-09RC reflectance array. |
| Line follower interval | `30 ms` | `lib/09 LineFollower/include/LineFollower.h` | Frequent enough for line correction during slow autonomous movement. |
| Line threshold | `blackThreshold = 400` | `LineFollower.h` | Separates dark line readings from lighter floor readings during line tests. |
| Line PD gains | `Kp = 0.07`, `Kd = 0.004` | `LineFollower.h` | Smooths steering and damps oscillation. |
| Door detected/open thresholds | `28 cm` / `70 cm` | `src/system.cpp` | Distinguishes a closed/near door from an open airlock path. |
| Obstacle threshold | `22 cm` | `src/system.cpp` | Triggers obstacle handling before collision. |
| Tunnel speed/time | `230`, `3500 ms` | `src/system.cpp` | Simple timed tunnel traverse after airlock acceptance. |
| Arena time limit | `4 minutes` | `src/system.cpp` | Forces return-to-base before the run expires. |
| Max seeds | `5` | `src/system.cpp` | Stops planting after the mission seed target is reached. |
| Planter half turn | `PLANTER_COUNTS_PER_REV / 2` | `lib/11 Planter/include/PlanterConfig.h` | Uses encoder counts for a controlled 180-degree planting cycle. |

## Integrated Behaviour Evidence

| Behaviour | Trigger | Expected result | Current status / limitation |
| --- | --- | --- | --- |
| Safety gate | Kill switch unsafe, missing heartbeat, or heartbeat timeout | Robot stops immediately, LED shows emergency, status reports `safety=0`. | Implemented at the top of `systemLoop()` before state movement. Must be demonstrated with wheels lifted first. |
| Start mission | Serial `S` or MQTT `type=start` while heartbeat is valid | State leaves `Idle` and requests exit airlock B. | Implemented. Requires server heartbeat `type=heartbeat enable=1`. |
| Airlock exit | Server accepts airlock B and front Lidar confirms door/open path | Robot follows line to door, waits, then tunnel wall-follows. | Implemented. Door thresholds may need final tuning on the real airlock. |
| Line following | IR array sees the dark line | `LineFollower` drives and steers using line error. | Implemented. `blackThreshold` should be checked on final floor lighting. |
| RFID soil query | RFID UID is detected in `GridDrive` | Robot sends `isFertile`; server reply updates the 9 x 9 grid. | Implemented. Route planning is limited to lower line-following rows. |
| Planting decision | Reply is fertile, not planted, and not blocked | Robot aligns, fine-adjusts, runs planter, increments seed count, and sends `seedPlanted`. | Implemented. Physical seed drop should be confirmed with the loaded hopper. |
| Return to base | Max seeds, time limit, return command, or emergency warning | Robot requests airlock A and enters base/complete state. | Implemented. Uses line following and tunnel timing; final arena placement may need tuning. |
| Stranded/revive | `type=stranded`, revive button, or `type=revive` | Robot stops in `Stranded`, then returns toward the airlock after revive. | Implemented. Button and server revive should both be shown during viva if asked. |
| Manual control | Serial `M`, then `W/A/S/D/Q/E/L/G/0` | Team can demonstrate motors, line follow, wall follow, and stop under local control. | Implemented. Still requires kill switch safe. |

## What Worked

- Code is split into mission control plus separate modules for drive, line following, wall following, RFID, planter, LED, kill switch, revive button, and MQTT.
- Safety is checked before autonomous state updates, so emergency/kill/heartbeat failures have higher priority than mission logic.
- Module test sketches exist for the main sensors, actuators, and communication path.
- Main loop state names are reported in status messages, which helps connect live robot behaviour to the code during the viva.

## What Did Not Work Or Needs Care

- WiFi credentials are currently stored in `include/WiFiHandlerConfig.h`; replace them before publishing the public repository.
- The arena grid router only treats the lower rows as routeable line-following rows (`GRID_LINE_ROW_LIMIT = 4`), so it is not a full free-space navigator.
- Airlock traversal still includes timed tunnel movement (`TUNNEL_TRAVERSE_MS` and `ENTRY_TUNNEL_TRAVERSE_MS`), so final airlock dimensions can affect reliability.
- IR threshold and Lidar distance thresholds should be rechecked on the final competition surface and lighting.

## Final-Day Checklist

1. Run `pio run -e giga_m7`.
2. Upload with `pio run -e giga_m7 --target upload`.
3. Open serial monitor at `115200`.
4. Confirm kill switch stops all motors with the robot lifted.
5. Confirm server heartbeat: `type=heartbeat enable=1`.
6. Confirm `type=register` and `type=status` are visible on the server.
7. Run short manual checks: `M`, `W`, `0`, `L`, `G`, `P`.
8. Place an RFID tag and confirm the robot sends `type=isFertile`.
9. Run one controlled planting cycle and confirm `type=seedPlanted`.
10. Start autonomous mode with `S` or MQTT `type=start`.
