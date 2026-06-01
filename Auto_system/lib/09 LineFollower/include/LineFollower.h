#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>

#include "IRSensor.h"
#include "MotorConfig.h"
#include "MotoronDrive.h"

struct LineFollowerConfig
{
    uint32_t controlIntervalMs = 30;
    uint32_t logIntervalMs = 200;
    int16_t baseSpeed = 220;
    int16_t searchTurn = 260;
    int16_t hardTurnSpeed = 450;
    int16_t hardTurnForward = 80;
    int16_t hardTurnError = 2200;
    int16_t maxTurn = 450;
    uint16_t blackThreshold = 400;
    int16_t sensorStep = 1000;
    float irKp = 0.07f;
    float irKd = 0.004f;
    float irDirection = 1.0f;
    bool requireEncoderSpeedControl = true;
    Stream* logOutput = nullptr;
};

class LineFollower
{
private:
    MotoronDrive& robot;
    IRSensor& sensors;
    LineFollowerConfig config;

    bool haveLastError;
    bool lineVisible;
    uint32_t lastControlMs;
    uint32_t lastLogMs;
    int16_t lastLineError;
    int16_t lastTurnValue;
    int16_t lastIrTurnValue;
    int8_t lastLineSide;
    float lastDerivativeValue;
    uint16_t lastContrastValue;

    int16_t clampTurn(float turn) const;
    void printLog(const char* mode, int16_t error, int16_t turn);

public:
    LineFollower(
        MotoronDrive& drive,
        IRSensor& lineSensors,
        const LineFollowerConfig& followerConfig = LineFollowerConfig()
    );

    void reset();
    void update();
    void update(int16_t speed);
    void setLogOutput(Stream* output);

    bool readLineError(int16_t& error, uint16_t& contrast);
    bool readLineError(int16_t& error);
    bool hasLine() const;
    int16_t lastError() const;
    int16_t lastTurn() const;
    int16_t lastIrTurn() const;
    int16_t lastEncoderTurn() const;
    float lastDerivative() const;
    uint16_t lastContrast() const;
    void getEncoderCounts(int32_t& fl, int32_t& fr, int32_t& rl, int32_t& rr)
        const;
};

#endif
