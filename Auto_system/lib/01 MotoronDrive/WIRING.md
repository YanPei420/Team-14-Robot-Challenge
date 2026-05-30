# MotoronDrive Wiring

This file records the intended wiring between each DG01D-E motor, the Motoron
controllers, and the Arduino GIGA encoder pins.

## DG01D-E Motor Connector

With the 6-pin motor connector oriented like the reference photo, the pins are:

| Connector position | Signal |
| --- | --- |
| 1 | Motor driver pin 1 |
| 2 | Motor driver pin 2 |
| 3 | Encoder `+` supply |
| 4 | Encoder `A` |
| 5 | Encoder `B` |
| 6 | Encoder `-` / GND |

The encoder `A` and `B` wires are the two middle encoder signal wires:

```text
1  Motor driver pin 1
2  Motor driver pin 2
3  Encoder +
4  Encoder A  -> Arduino GIGA A pin below
5  Encoder B  -> Arduino GIGA B pin below
6  Encoder -  -> GND
```

## Per-Wheel A/B Pin Map

These are the expected encoder A/B pins from
`include/MotorConfig.h`.

| Wheel | Motoron board | Motoron channel | Encoder A | Encoder B |
| --- | --- | --- | --- | --- |
| Front left (`FL`) | Front Motoron `0x10` | Channel `1` | `D25` | `D26` |
| Front right (`FR`) | Front Motoron `0x10` | Channel `2` | `D22` | `D23` |
| Rear left (`RL`) | Rear Motoron `0x11` | Channel `1` | `D28` | `D29` |
| Rear right (`RR`) | Rear Motoron `0x11` | Channel `2` | `D27` | `D28` |

Note: this map uses `D28` for both `RL` encoder A and `RR` encoder B. Two
encoder signals normally should not share one GIGA input pin. If both rear
encoders are connected, verify whether `RR` encoder B should be a different
pin.

## Encoder Power

All four encoders need:

| Encoder wire | Connect to |
| --- | --- |
| Encoder `+` | Logic supply used by the encoder board |
| Encoder `-` | Arduino GIGA GND, shared with motor controllers |

Do not swap encoder supply with motor power. The encoder `A` and `B` signals
must share ground with the GIGA.

## Current Diagnostic Finding

During serial testing, command `3` (`rear-left` motor command) caused the
`FR` encoder pins (`D24`/`D25`) to change. That means one of these is true:

| Symptom | Likely cause |
| --- | --- |
| Physical rear-left wheel turns, but `FR` encoder changes | Rear-left encoder A/B wires are plugged into `D24`/`D25`; move them to `D26`/`D27`, or update the config map. |
| Physical front-right wheel turns when command `3` is sent | Motor output wiring or Motoron channel labels are swapped. |
| Wheel turns but no `poll_edges` changes | Encoder A/B/supply/GND for that wheel is not reaching the GIGA pins in the table. |

Use `test/encoder_test.cpp` with these commands:

```text
M    initialize Motoron
1    run FL motor briefly
2    run FR motor briefly
3    run RL motor briefly
4    run RR motor briefly
P    print encoder data
```

For each motor command, the matching wheel's `poll_edges` should increase.
