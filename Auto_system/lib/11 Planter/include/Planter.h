#pragma once

#include <Arduino.h>

#include "Encoder.h"
#include "MotoronDrive.h"
#include "PlanterConfig.h"

class Planter
{
public:
    enum class State : uint8_t
    {
        Idle,
        Running,
        Complete,
        Timeout
    };

    explicit Planter(MotoronDrive& drive);

    void begin();
    void startCycle();
    void stop();
    bool update();

    bool isRunning() const;
    bool isComplete() const;
    bool hasTimedOut() const;
    State state() const;
    long position() const;
    long targetCounts() const;

private:
    MotoronDrive& drive_;
    Encoder encoder_;
    State state_;
    long startPosition_;
    long targetCounts_;
    uint32_t startedMs_;

    long absoluteLong(long value) const;
    void writeMotor(int16_t speed, bool immediate);
    void restoreDriveControl();
};
