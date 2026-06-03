# Team 14 — Robot Challenge

Repository for the Team 14 autonomous robot system, built for the Robot Challenge competition. The codebase is primarily C++ with supporting HTML tooling and C drivers.

> **GitHub:** https://github.com/YanPei420/Team-14-Robot-Challenge

---

## Repository structure

```
Team-14-Robot-Challenge/
├── Auto_system/                       # Main autonomous system firmware
│   └── tools/
│       ├── Software_overview.md       # Layered software architecture diagram
│       └── README.md                  # Detailed auto system documentation
├── Components/                        # Reusable hardware driver modules
├── Sensors/                           # Sensor interface code
├── Flowcharts/
│   └── System_Flowchart.md            # FSM — the definitive system flowchart
├── Testing and Calibration Evidence/  # Test logs and calibration records
├── docs/                              # Additional project documentation
├── Bill_of_Materials.md               # Links to the full hardware BoM (SharePoint)
├── Logs.md                            # Development and session logs
└── Pinout.md                          # Pin assignment reference
```

---

## Auto_system

The core autonomous firmware stack. Contains the finite state machine (FSM), navigator, MQTT communication layer, sensor event handling, and all robot behaviour logic.

The `tools/` subfolder contains two key orientation documents:

- **Software_overview.md** — a layered diagram of the full software architecture, showing how the runtime loop, safety layer, and mission states relate to one another. Start here for a high-level understanding of the system.
- **README.md** — a detailed breakdown of the FSM logic specific to the auto system, covering file responsibilities, state transitions, timing constants, and the runtime event handling order.

---

## Flowcharts

Contains **System_Flowchart.md**, the definitive reference for the robot's state machine behaviour. This is the primary document to consult when reasoning about robot behaviour or debugging state transitions.

It covers:

- The safety layer (`Normal` / `EmergencyStop`)
- Top-level mission states (`Base`, `Grid`, `Stranded`, `Revived`)
- Sub-state sequences for `ExitBase`, `ReturnHome`, `ExploreGrid`, `Align`, and `Plant`
- Transition triggers, automatic events, and timing constants
- Remote event mappings (MQTT messages → FSM effects)
- The full typical mission sequence from start to `InsideBase`

---

## Components

Driver-level modules for individual hardware components. These are consumed by the auto system firmware and can be developed and tested independently.

---

## Sensors

Interface code for the robot's sensor suite, including Lidar, IR line sensors, and the RFID reader.

---

## Testing and Calibration Evidence

Logs, results, and supporting evidence from hardware testing and sensor calibration sessions. Useful for verifying that sensor thresholds and timing constants in the FSM match real-world behaviour.

---

## Bill_of_Materials.md

Links to the full hardware Bill of Materials, hosted as an Excel spreadsheet on SharePoint. Lists all components, quantities, and sourcing information for the robot build.

→ [Open BoM (SharePoint)](https://liveuclac-my.sharepoint.com/:x:/r/personal/zcabypz_ucl_ac_uk/Documents/Bom%20for%20interim%20report-%20team%2014.xlsx?d=w970fc6ec0cbc4649b9f93f0a8302760a&csf=1&web=1&e=DEkvMu)

---

## Pinout.md

Pin assignment table for the robot's microcontrollers. Consult this when modifying wiring, adding new peripherals, or cross-referencing hardware against firmware configuration.

---

## Logs.md

Running development log tracking changes, issues encountered, and session notes across the project lifetime.

---

## Quick reference

| What you need | Where to look |
|---|---|
| System state machine (FSM) | `Flowcharts/System_Flowchart.md` |
| Software architecture overview | `Auto_system/tools/Software_overview.md` |
| Auto system detailed docs | `Auto_system/tools/README.md` |
| Bill of materials | `Bill_of_Materials.md` |
| Pin assignments | `Pinout.md` |
| Development log | `Logs.md` |
