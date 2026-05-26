#if defined(CORE_CM7)

#include "M7DriveProxy.h"

#include <RPC.h>

namespace
{
constexpr uint32_t DRIVE_RPC_TIMEOUT_MS = 250;
constexpr uint8_t M4_READY_ATTEMPTS = 20;
constexpr uint32_t M4_READY_RETRY_DELAY_MS = 100;
} // namespace

bool M7DriveProxy::begin()
{
    RPC.setTimeout(DRIVE_RPC_TIMEOUT_MS);

    for (uint8_t attempt = 0; attempt < M4_READY_ATTEMPTS; ++attempt)
    {
        if (rpcCallOk("m4_motor_begin"))
        {
            ready_ = true;
            return true;
        }

        delay(M4_READY_RETRY_DELAY_MS);
    }

    ready_ = false;
    return false;
}

void M7DriveProxy::set_all(
    int16_t frontLeft,
    int16_t frontRight,
    int16_t rearLeft,
    int16_t rearRight
)
{
    if (!ready_)
    {
        return;
    }

    RPC.send(
        "m4_motor_set_all",
        static_cast<int>(frontLeft),
        static_cast<int>(frontRight),
        static_cast<int>(rearLeft),
        static_cast<int>(rearRight)
    );
}

void M7DriveProxy::drive(int16_t vx, int16_t vy, int16_t w)
{
    if (!ready_)
    {
        return;
    }

    RPC.send(
        "m4_motor_drive",
        static_cast<int>(vx),
        static_cast<int>(vy),
        static_cast<int>(w)
    );
}

void M7DriveProxy::stop_all()
{
    if (!ready_)
    {
        return;
    }

    rpcCallOk("m4_motor_stop_all");
}

bool M7DriveProxy::rpcCallOk(const char* name)
{
    const int result = RPC.call(name).as<int>();
    return !RPC.timedOut() && result == 1;
}

#endif
