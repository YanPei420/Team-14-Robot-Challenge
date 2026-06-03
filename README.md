# Team 14 — Robot Challenge

Repository for the Team 14 autonomous robot system, built for the Term 3 Robotics Challenge 2026. The codebase is primarily C++ with supporting HTML tooling and C drivers.

> **GitHub:** https://github.com/YanPei420/Team-14-Robot-Challenge

---

## Repository structure

```
Team-14-Robot-Challenge/
├── Auto_system/                       # Main autonomous system firmware
│   ├── docs/
│   │   ├── software_overview.md       # Layered software architecture diagram
│   │   ├── system_flowchart.md        # FSM — the definitive system flowchart
│   │   ├── testing_calibration.md     # Testing and calibration notes
│   │   ├── website.md                 # Website documentation
│   │   ├── ABX00063-datasheet.pdf
│   │   ├── ABX00063-full-pinout.pdf
│   │   ├── DS_16413_DG01D_E_Motor_with_Encoder.pdf
│   │   ├── Pololu - QTR-HD-09RC Reflectance Sensor Array.pdf
│   │   ├── Pololu Motoron Motor Controller User's Guide.pdf
│   │   ├── Term3_Robotics_Challenge_2026_main.pdf
│   │   ├── WS1850S QFN-32.pdf
│   │   ├── adafruit-tdk-invensense-icm-20948-9-dof-imu.pdf
│   │   ├── mecanum wheels info.pdf
│   │   ├── micro motor N20.pdf
│   │   └── rfid_product_manual.pdf
│   ├── tool/
│   │   ├── README.md                  # Auto system build & usage guide
│   │   ├── AGENT.md
│   │   └── platformio.ini
│   ├── include/
│   ├── lib/
│   ├── src/                           # C++ firmware source
│   └── test/
├── Components/                        # Reusable hardware driver modules
├── Sensors/                           # Sensor interface code
├── Testing and Calibration Evidence/  # Test logs and calibration records
├── docs/                              # Additional project documentation
├── Bill_of_Materials.md               # Links to the full hardware BoM (SharePoint)
├── Logs.md                            # Development and session logs
└── Pinout.md                          # Pin assignment reference
```

---

## Auto_system

The core autonomous firmware stack. Contains the finite state machine (FSM), navigator, MQTT communication layer, sensor event handling, and all robot behaviour logic. Source code lives in `src/`, with headers in `include/` and libraries in `lib/`.

### `docs/`

The primary documentation and reference folder. Contains two key system documents alongside all component datasheets:

- **system_flowchart.md** — the definitive reference for the robot's state machine behaviour, referred to as the system flowchart. Consult this when reasoning about robot behaviour or debugging state transitions. Covers the safety layer, all mission states and sub-states (`ExitBase`, `ReturnHome`, `ExploreGrid`, `Align`, `Plant`), transition triggers, timing constants, remote event mappings, and the full typical mission sequence.
- **software_overview.md** — a layered diagram of the full software architecture, showing how the runtime loop, safety layer, and mission states relate to one another. Start here for a high-level understanding of the system.
- **testing_calibration.md** — notes and records from hardware testing and sensor calibration.
- **website.md** — project website documentation.

The remaining files in `docs/` are datasheets for all major hardware components used in the build.

### `tool/`

Build tooling and platform configuration:

- **README.md** — detailed guide to the auto system: building, flashing, and working with the firmware.
- **platformio.ini** — PlatformIO project configuration.
- **AGENT.md** — agent configuration notes.

---

## Components

Driver-level modules for individual hardware components, used by the auto system firmware and developed independently.

---

## Sensors

Interface code for the robot's sensor suite, including Lidar, IR line sensors, and the RFID reader.

---

## Testing and Calibration Evidence

Logs, results, and supporting evidence from hardware testing and sensor calibration sessions.

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
| System flowchart (FSM) | `Auto_system/docs/system_flowchart.md` |
| Software architecture overview | `Auto_system/docs/software_overview.md` |
| Auto system build & usage guide | `Auto_system/tool/README.md` |
| Component datasheets | `Auto_system/docs/` |
| Bill of materials | `Bill_of_Materials.md` |
| Pin assignments | `Pinout.md` |
| Development log | `Logs.md` |
