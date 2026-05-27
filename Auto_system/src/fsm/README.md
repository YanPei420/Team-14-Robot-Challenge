# FSM layout

`RobotFSM` is declared in `src/RobotFSM.h`. The implementation is split by
state-machine responsibility:

- `Core.cpp`: constructor, top-level update, query helpers, common helpers
- `Safety.cpp`: emergency stop transitions
- `Names.cpp`: state name formatting and state logging
- `base/Mission_base.cpp`: base, exit tunnel, and return-home events/transitions
- `grid/Mission_grid.cpp`: grid exploration, RFID query, align, plant, and recovery events
- `FSMconfig.h`: speeds and timing constants used by `RobotFSM`

The M7 application wrapper around the FSM lives here too:

- `FSM_main.cpp`: setup/loop orchestration for WiFi, safety gate, sensors, and FSM update
- `FSM_remote_events.*`: MQTT payload parsing into FSM events
- `FSM_auto_events.*`: calibration hooks for automatic door, arena, base, stranded, and RFID map detection

Keep direct motor controller code on M4. `RobotFSM` should only talk to the
`RobotDrive` interface.
