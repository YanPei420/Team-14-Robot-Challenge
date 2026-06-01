# MotoronDrive

MotoronDrive is the robot chassis drive wrapper for two Pololu Motoron I2C
motor controllers on the Arduino Giga R1 WiFi.

The class hides the repeated Motoron setup code, keeps all motor addresses and
direction signs in one config file, clamps speed commands, and exposes simple
chassis movement helpers such as `forward()`, `right()`, and `stop_all()`.

## Files

| File | Purpose |
| --- | --- |
| `include/MotorConfig.h` | Motoron addresses, channels, speed limits, acceleration, deceleration, and motor direction signs. |
| `include/MotoronDrive.h` | Public `MotoronDrive` API. |
| `include/MotorEncoder.h` | Public DG01D-E quadrature encoder API. |
| `src/MotoronDrive.cpp` | Motoron initialization, wheel output, mecanum drive math, stop, raw debug, and status helpers. |
| `src/MotorEncoder.cpp` | Polling wheel encoder counting, revolutions, and RPM sampling, with optional interrupt mode. |
| `WIRING.md` | Motoron, DG01D-E motor, encoder, and Arduino GIGA wiring notes. |

## Hardware Mapping

The controllers use `Wire1`.

| Device | I2C address |
| --- | --- |
| Front Motoron board | `16` (`0x10`) |
| Rear Motoron board | `17` (`0x11`) |

| Wheel | Board | Motoron channel |
| --- | --- | --- |
| Front left | Front | `1` |
| Front right | Front | `2` |
| Rear left | Rear | `1` |
| Rear right | Rear | `2` |

## Encoder Mapping

DG01D-E encoder pins are read on both A and B channels, so the default count
mode is x4 quadrature counting. On Arduino GIGA the default backend is polling
(`MOTOR_ENCODER_USE_INTERRUPTS = false`) because the Mbed interrupt path can
fault on some D22-D29 pins during startup.

| Wheel | Encoder A pin | Encoder B pin | Direction sign |
| --- | --- | --- | --- |
| Front left | `D24` | `D25` | `-1` |
| Front right | `D22` | `D23` | `1` |
| Rear left | `D26` | `D27` | `-1` |
| Rear right | `D28` | `D29` | `1` |

See `WIRING.md` for the DG01D-E 6-pin connector order. In the reference motor
photo, encoder `A` is connector pin `4` and encoder `B` is connector pin `5`.

The DG01D-E datasheet gives a `1:48` gear ratio and `6` encoder pulses per
motor revolution. With x4 quadrature counting, the default output-shaft count
constant is:

```cpp
MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV = 6 * 48 * 4; // 1152
```

If measured wheel counts differ, update `MOTOR_ENCODER_COUNTS_PER_OUTPUT_REV`
or pass a custom count value to `MotoronDriveEncoders`.

## Direction Mapping

Logical wheel speeds are converted to raw Motoron speeds using the direction
signs in `MotorConfig.h`.

| Wheel | Direction sign |
| --- | --- |
| Front left | `-1` |
| Front right | `1` |
| Rear left | `-1` |
| Rear right | `1` |

So:

```cpp
Robot.forward(500);
```

sends these raw Motoron speeds:

| Motoron output | Speed |
| --- | --- |
| Front channel 1 | `-500` |
| Front channel 2 | `500` |
| Rear channel 1 | `-500` |
| Rear channel 2 | `500` |

## Default Settings

| Setting | Value |
| --- | --- |
| `MOTOR_MAX_SPEED` | `800` |
| `MOTOR_MAX_ACCELERATION` | `300` |
| `MOTOR_MAX_DECELERATION` | `600` |
| `MOTOR_COMMAND_TIMEOUT_MS` | `1000` |

All normal wheel commands are clamped to `[-maxSpeed, maxSpeed]`.

## Basic Usage

```cpp
#include <Arduino.h>

#include "MotorConfig.h"
#include "MotorEncoder.h"
#include "MotoronDrive.h"

MotoronDrive Robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
MotoronDriveEncoders Encoders;

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    Robot.begin();
    Encoders.begin();
}

void loop()
{
    Robot.forward(500);
    delay(1000);
}
```

`begin()` calls `Wire1.begin()`, attaches both Motoron controllers to `Wire1`,
reinitializes them, disables CRC, clears reset flags, sets command timeout,
sets acceleration/deceleration for channels 1 and 2, and stops all motors.

## Movement API

| Method | Description |
| --- | --- |
| `forward(speed)` | Drive forward. |
| `backward(speed)` | Drive backward. |
| `left(speed)` | Strafe left. |
| `right(speed)` | Strafe right. |
| `rotate_left(speed)` | Rotate left. |
| `rotate_right(speed)` | Rotate right. |
| `drive(vx, vy, w)` | Mecanum drive command. `vx` is forward/back, `vy` is right/left, `w` is rotation. |
| `stop()` | Alias for `stop_all()`. |
| `stop_all()` | Immediately sets all four channels to zero with `setSpeedNow()`. |

`drive(vx, vy, w)` uses this wheel math:

```cpp
frontLeft  = vx - vy - w;
frontRight = vx + vy + w;
rearLeft   = vx + vy - w;
rearRight  = vx - vy + w;
```

If any computed wheel speed is above `maxSpeed`, all four wheel speeds are
scaled down proportionally.

## Wheel API

| Method | Description |
| --- | --- |
| `set_front_left(speed)` | Set the logical front-left wheel speed. |
| `set_front_right(speed)` | Set the logical front-right wheel speed. |
| `set_rear_left(speed)` | Set the logical rear-left wheel speed. |
| `set_rear_right(speed)` | Set the logical rear-right wheel speed. |
| `set_all(fl, fr, rl, rr)` | Set all four logical wheel speeds. |
| `get_wheel_speeds(fl, fr, rl, rr)` | Read the last commanded logical wheel speeds. |
| `set_max_speed(maxSpeed)` | Set the clamp limit used by normal commands. |
| `get_max_speed()` | Read the current clamp limit. |

## Encoder Speed Control

`MotoronDrive` can use the wheel encoders as a closed-loop speed controller.
By default `begin()` starts the encoder readers and enables closed-loop speed
control. Normal movement calls such as `drive()`, `forward()`, and
`rotate_left()` set logical wheel targets, then the library compares target RPM
with encoder RPM and increases or decreases each wheel output as needed.

Call `update()` or `update_encoder_speed_control()` frequently from `loop()` so
the correction can keep running while the robot is moving.

```cpp
MotoronDrive Robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

void setup()
{
    Robot.begin();
    Robot.set_max_speed(MOTOR_MAX_SPEED);

    if (!Robot.encoder_speed_control_ready())
    {
        Serial.println("encoder speed control failed");
    }
}

void loop()
{
    Robot.forward(300);
    Robot.update();
}
```

| Method | Description |
| --- | --- |
| `begin_encoder_speed_control(config)` | Starts all four encoder readers and enables closed-loop speed control. |
| `set_encoder_speed_control_enabled(enabled)` | Enables or disables the closed-loop correction after the encoders have started. |
| `update_encoder_speed_control()` | Samples encoder RPM and updates Motoron outputs when the control interval has elapsed. |
| `update()` | Short alias for `update_encoder_speed_control()`. |
| `reset_encoder_speed_control()` | Clears PID state and encoder counts. |
| `set_encoder_speed_control_config(config)` | Updates the speed-control gains and limits. |
| `get_applied_wheel_speeds(fl, fr, rl, rr)` | Reads the corrected logical outputs currently being sent to the wheels. |
| `get_encoder_rpm(fl, fr, rl, rr)` | Reads the most recent measured wheel RPM values. |
| `get_target_rpm(fl, fr, rl, rr)` | Reads the RPM targets derived from logical wheel speeds. |

The default controller maps `MOTOR_MAX_SPEED` to
`MOTOR_SPEED_CONTROL_MAX_WHEEL_RPM`, then applies PID correction:

```cpp
output = target_speed + PID(target_rpm - measured_rpm)
```

Tune `MOTOR_SPEED_CONTROL_MAX_WHEEL_RPM`, `MOTOR_SPEED_CONTROL_KP`,
`MOTOR_SPEED_CONTROL_KI`, and `MOTOR_SPEED_CONTROL_MAX_CORRECTION` in
`MotorConfig.h` if the wheels overshoot, undershoot, or oscillate.

## Raw Debug API

Raw methods send speeds directly to Motoron channels without applying wheel
direction signs. They still clamp to the configured maximum speed.

| Method | Description |
| --- | --- |
| `raw_front(motor1, motor2, immediate)` | Set both front-board Motoron channels directly. |
| `raw_rear(motor1, motor2, immediate)` | Set both rear-board Motoron channels directly. |
| `raw_front_motor(channel, speed, immediate)` | Set one front-board channel directly. |
| `raw_rear_motor(channel, speed, immediate)` | Set one rear-board channel directly. |

When `immediate` is `true`, raw methods use `setSpeedNow()`. Otherwise they use
`setSpeed()`.

Calling any raw method disables encoder speed control so the raw command is not
overwritten by the closed-loop controller.

## Status Helpers

| Method | Description |
| --- | --- |
| `clear_status_flags()` | Clears reset, latched status, and motor fault flags on both Motoron boards. |
| `print_status(Stream& output)` | Prints address, status flags, and last I2C error for both boards. |

## Encoder API

| Method | Description |
| --- | --- |
| `MotoronDriveEncoders::begin()` | Starts all four encoder readers. |
| `poll()` | Updates encoder counts when using the default polling backend. |
| `reset_counts()` | Resets all four wheel counts to zero. |
| `get_counts(fl, fr, rl, rr)` | Reads signed x4 quadrature counts for all wheels. |
| `get_revolutions(fl, fr, rl, rr)` | Converts counts to output-shaft revolutions. |
| `sample_rpm(fl, fr, rl, rr)` | Samples signed RPM since the previous call. |
| `front_left()` / `front_right()` / `rear_left()` / `rear_right()` | Access one `MotorEncoder` directly. |

Single-encoder helpers include `read_count()`, `read_and_reset()`,
`reset_count()`, `read_revolutions()`, `sample_rpm()`,
`get_counts_per_revolution()`, and `set_counts_per_revolution()`.
