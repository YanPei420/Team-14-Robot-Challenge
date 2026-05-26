# M7 core responsibilities

Based on the Arduino GIGA R1 WiFi datasheet for ABX00063, the Cortex-M7 core is
the 480 MHz application core and the board includes a Murata Wi-Fi/Bluetooth
module. This firmware uses M7 for high-level work:

- booting M4 through Arduino RPC
- WiFi/MQTT communication through `MiniMessenger`
- remote safety heartbeat handling
- mission FSM orchestration in `src/fsm/FSM_main.cpp`
- RPC commands to the M4 chassis service

Keep direct motor controller refresh and encoder handling on M4.
