#if defined(CORE_CM4)

#include "M4MotorService.h"

#include <Arduino.h>
#include <MotorEncoder.h>
#include <MotoronDrive.h>
#include <RPC.h>

namespace
{
constexpr uint32_t CHASSIS_REFRESH_INTERVAL_MS = 50;

MotoronDrive robot;
MotoronDriveEncoders encoders;

bool initialized = false;
bool encodersReady = false;
uint32_t lastRefreshMs = 0;

enum class MotionMode : uint8_t
{
    Stopped,
    DriveVector,
    WheelSpeeds
};

MotionMode motionMode = MotionMode::Stopped;
int16_t targetVx = 0;
int16_t targetVy = 0;
int16_t targetW = 0;
int16_t targetFrontLeft = 0;
int16_t targetFrontRight = 0;
int16_t targetRearLeft = 0;
int16_t targetRearRight = 0;

int16_t clampRpcSpeed(int value)
{
    if (value > INT16_MAX)
    {
        return INT16_MAX;
    }

    if (value < INT16_MIN)
    {
        return INT16_MIN;
    }

    return static_cast<int16_t>(value);
}

void refreshMotion()
{
    if (!initialized)
    {
        return;
    }

    switch (motionMode)
    {
        case MotionMode::Stopped:
            break;

        case MotionMode::DriveVector:
            robot.drive(targetVx, targetVy, targetW);
            break;

        case MotionMode::WheelSpeeds:
            robot.set_all(
                targetFrontLeft,
                targetFrontRight,
                targetRearLeft,
                targetRearRight
            );
            break;
    }

    lastRefreshMs = millis();
}

int motorBegin()
{
    if (initialized)
    {
        return 1;
    }

    const bool motorsReady = robot.begin();
    robot.stop_all();

    encodersReady = encoders.begin();
    encoders.reset_counts();

    initialized = motorsReady;
    motionMode = MotionMode::Stopped;
    lastRefreshMs = millis();

    return motorsReady ? 1 : 0;
}

int motorDrive(int vx, int vy, int w)
{
    if (!initialized && motorBegin() == 0)
    {
        return 0;
    }

    targetVx = clampRpcSpeed(vx);
    targetVy = clampRpcSpeed(vy);
    targetW = clampRpcSpeed(w);
    motionMode = MotionMode::DriveVector;
    refreshMotion();

    return 1;
}

int motorSetAll(int frontLeft, int frontRight, int rearLeft, int rearRight)
{
    if (!initialized && motorBegin() == 0)
    {
        return 0;
    }

    targetFrontLeft = clampRpcSpeed(frontLeft);
    targetFrontRight = clampRpcSpeed(frontRight);
    targetRearLeft = clampRpcSpeed(rearLeft);
    targetRearRight = clampRpcSpeed(rearRight);
    motionMode = MotionMode::WheelSpeeds;
    refreshMotion();

    return 1;
}

int motorStopAll()
{
    if (!initialized && motorBegin() == 0)
    {
        return 0;
    }

    motionMode = MotionMode::Stopped;
    targetVx = 0;
    targetVy = 0;
    targetW = 0;
    targetFrontLeft = 0;
    targetFrontRight = 0;
    targetRearLeft = 0;
    targetRearRight = 0;
    robot.stop_all();
    lastRefreshMs = millis();

    return 1;
}

int motorEncodersReady()
{
    return encodersReady ? 1 : 0;
}
} // namespace

namespace M4MotorService
{
void setup()
{
    RPC.begin();

    RPC.bind("m4_motor_begin", motorBegin);
    RPC.bind("m4_motor_drive", motorDrive);
    RPC.bind("m4_motor_set_all", motorSetAll);
    RPC.bind("m4_motor_stop_all", motorStopAll);
    RPC.bind("m4_motor_encoders_ready", motorEncodersReady);

    motorBegin();
}

void loop()
{
    if (
        initialized &&
        motionMode != MotionMode::Stopped &&
        (millis() - lastRefreshMs) >= CHASSIS_REFRESH_INTERVAL_MS
    )
    {
        refreshMotion();
    }

    delay(1);
}
} // namespace M4MotorService

#endif
