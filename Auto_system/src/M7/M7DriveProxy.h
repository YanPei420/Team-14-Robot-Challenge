#pragma once

#include "../RobotDrive.h"

class M7DriveProxy : public RobotDrive
{
public:
    bool begin() override;
    void set_all(
        int16_t frontLeft,
        int16_t frontRight,
        int16_t rearLeft,
        int16_t rearRight
    ) override;
    void drive(int16_t vx, int16_t vy, int16_t w) override;
    void stop_all() override;

private:
    bool rpcCallOk(const char* name);
    bool ready_ = false;
};
