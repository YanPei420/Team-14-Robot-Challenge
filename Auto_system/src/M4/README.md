# M4 core responsibilities

Based on the Arduino GIGA R1 WiFi datasheet for ABX00063, the Cortex-M4 core is
the lower-power 240 MHz core with FPU support. This firmware uses it for
time-sensitive, low-level robot hardware:

- Motoron chassis control on `Wire1`
- wheel encoder GPIO interrupts
- periodic motor command refresh so the Motoron timeout does not stop normal
  FSM movement
- RPC endpoints consumed by the M7 core

Keep networking, MQTT, mission policy, and other high-level behavior on M7.

## TODO: motor PD control

Motor PD control belongs on M4 because it needs encoder feedback and should run
close to the motor refresh loop.

- [ ] Add encoder speed measurement for each wheel
- [ ] Track target wheel speed from `m4_motor_drive` and `m4_motor_set_all`
- [ ] Apply PD correction before sending commands to Motoron
- [ ] Add tuning constants for proportional and derivative gains
- [ ] Add simple serial/RPC debug output for target speed, measured speed, and
  correction value
