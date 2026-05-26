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
