# System Flowchart

This document describes the current simple sequential control flow in `src/system.cpp`. The active firmware no longer uses the old FSM implementation. Competition behavior is driven by `RunState` and the matching `update...()` functions.

## Main Runtime Loop

```mermaid
flowchart TD
    A["systemLoop()"] --> B["messenger.loop()"]
    B --> C["pollSerialCommands()"]
    C --> D["sendRegister()"]
    D --> E["update kill switch / revive button / lidar"]
    E --> F{"safetyAllowed()?"}

    F -- "No" --> G["robot.stop_all()"]
    G --> H["LED emergency"]
    H --> I["sendStatus()"]
    I --> Z["return"]

    F -- "Yes" --> J["handleGlobalRequests()"]
    J --> K["updateState()"]
    K --> L["updateStatusLed()"]
    L --> M["sendStatus()"]
    M --> N["next loop"]
```

## Safety Gate

```mermaid
flowchart TD
    A["safetyAllowed()"] --> B{"motorReady?"}
    B -- "No" --> X["blocked"]
    B -- "Yes" --> C{"killSwitch.isSafe()?"}
    C -- "No" --> X
    C -- "Yes" --> D{"heartbeatOk()?"}
    D -- "No" --> X
    D -- "Yes" --> Y["motion allowed"]

    H["heartbeatOk()"] --> I{"remoteSafetyEnabled?"}
    I -- "No" --> HX["false"]
    I -- "Yes" --> J{"lastHeartbeatMs != 0?"}
    J -- "No" --> HX
    J -- "Yes" --> K{"heartbeat age <= WIFI_HEARTBEAT_TIMEOUT_MS?"}
    K -- "No" --> HX
    K -- "Yes" --> HY["true"]
```

## Competition State Flow

```mermaid
flowchart TD
    Idle["Idle<br/>Stop and wait"]
    ExitRequest["ExitRequest<br/>Request airlock B"]
    ExitLineToDoor["ExitLineToDoor<br/>Follow line to exit door"]
    ExitWaitDoor["ExitWaitDoor<br/>Stop until door is open"]
    ExitTunnel["ExitTraverseTunnel<br/>Drive through tunnel"]
    GridDrive["GridDrive<br/>Follow line / avoid obstacle / scan RFID"]
    SoilQuery["SoilQuery<br/>Ask server if tag is fertile"]
    AlignSearch["AlignSearch<br/>Stop and settle on RFID"]
    FineAdjust["FineAdjust<br/>Move forward slowly"]
    PlantOpen["PlantOpen<br/>Open hopper servo"]
    PlantDrop["PlantDrop<br/>Wait for seed drop"]
    PlantVerify["PlantVerify<br/>Close hopper and count seed"]
    ReturnToAirlock["ReturnToAirlock<br/>Follow line back to entry door"]
    EntryRequest["EntryRequest<br/>Request airlock A"]
    EntryWaitDoor["EntryWaitDoor<br/>Stop until entry door is open"]
    EntryTunnel["EntryTraverseTunnel<br/>Drive through tunnel"]
    InsideBase["InsideBase<br/>Stop and send missionComplete"]
    Stranded["Stranded<br/>Stop until revived"]
    Finished["Finished<br/>Stop"]

    Idle -->|"startRequested"| ExitRequest
    ExitRequest -->|"openAirlockReply accepted B"| ExitLineToDoor
    ExitLineToDoor -->|"lidar door near stable"| ExitWaitDoor
    ExitWaitDoor -->|"lidar door open stable"| ExitTunnel
    ExitTunnel -->|"TUNNEL_TRAVERSE_MS elapsed"| GridDrive

    GridDrive -->|"RFID detected"| SoilQuery
    GridDrive -->|"seedsPlanted >= MAX_SEEDS"| ReturnToAirlock
    GridDrive -->|"ARENA_TIME_LIMIT_MS elapsed"| ReturnToAirlock
    GridDrive -->|"return / emergency_warning"| ReturnToAirlock

    SoilQuery -->|"fertile and not planted"| AlignSearch
    SoilQuery -->|"infertile / planted / timeout"| GridDrive

    AlignSearch -->|"ALIGN_SEARCH_MS elapsed"| FineAdjust
    FineAdjust -->|"FINE_ADJUST_MS elapsed"| PlantOpen
    PlantOpen -->|"HOPPER_OPEN_MS elapsed"| PlantDrop
    PlantDrop -->|"DROP_SEED_MS elapsed"| PlantVerify
    PlantVerify -->|"seeds remain"| GridDrive
    PlantVerify -->|"MAX_SEEDS reached"| ReturnToAirlock

    ReturnToAirlock -->|"lidar door near stable"| EntryRequest
    ReturnToAirlock -->|"base_reached message"| InsideBase
    EntryRequest -->|"openAirlockReply accepted A"| EntryWaitDoor
    EntryWaitDoor -->|"door open stable or accepted A"| EntryTunnel
    EntryTunnel -->|"line confirmed or fallback timeout"| InsideBase

    InsideBase -->|"startRequested"| ExitRequest
    Finished -->|"startRequested"| ExitRequest

    GridDrive -->|"stranded message"| Stranded
    SoilQuery -->|"stranded message"| Stranded
    AlignSearch -->|"stranded message"| Stranded
    FineAdjust -->|"stranded message"| Stranded
    PlantOpen -->|"stranded message"| Stranded
    PlantDrop -->|"stranded message"| Stranded
    PlantVerify -->|"stranded message"| Stranded
    ReturnToAirlock -->|"stranded message"| Stranded
    Stranded -->|"revive button or revive message"| ReturnToAirlock
```

## Remote And Serial Inputs

```mermaid
flowchart TD
    A["Incoming command"] --> B{"source"}

    B -->|"MQTT"| C["handleRemotePayload()"]
    B -->|"Serial"| D["pollSerialCommands()"]

    C --> C1["heartbeat enable=1<br/>remote safety on"]
    C --> C2["heartbeat enable=0 / stop / emergency / disable<br/>remote safety off and stop"]
    C --> C3["start<br/>startRequested = true"]
    C --> C4["openAirlockReply<br/>set exit/entry accepted flag"]
    C --> C5["isFertileReply / rfid<br/>finish soil query"]
    C --> C6["return / emergency_warning<br/>request ReturnToAirlock"]
    C --> C7["stranded<br/>enter Stranded"]
    C --> C8["revive<br/>leave Stranded"]
    C --> C9["base_reached<br/>enter InsideBase"]

    D --> D1["S<br/>startRequested = true"]
    D --> D2["R<br/>remoteReturnRequested = true"]
    D --> D3["X<br/>remoteSafetyEnabled = false and stop"]
```

## Outputs To Server

```mermaid
flowchart TD
    A["Robot messages"] --> B["register<br/>periodic identity"]
    A --> C["status<br/>state, seeds, safety"]
    A --> D["openAirlockB<br/>exit request"]
    A --> E["openAirlockA<br/>entry request"]
    A --> F["isFertile<br/>RFID soil query"]
    A --> G["seedPlanted<br/>after PlantVerify"]
    A --> H["missionComplete<br/>after InsideBase"]
```
