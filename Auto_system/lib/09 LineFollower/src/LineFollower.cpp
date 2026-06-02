#include "LineFollower.h"

#include "IRConfig.h"

LineFollower::LineFollower(
    MotoronDrive& drive,
    IRSensor& lineSensors,
    const LineFollowerConfig& followerConfig
)
    : robot(drive),
      sensors(lineSensors),
      config(followerConfig),
      haveLastError(false),
      lineVisible(false),
      lastControlMs(0),
      lastLogMs(0),
      lastLineError(0),
      lastTurnValue(0),
      lastIrTurnValue(0),
      lastLineSide(1),
      lastDerivativeValue(0.0f),
      lastContrastValue(0)
{
}

void LineFollower::reset()
{
    haveLastError = false;
    lineVisible = false;
    lastControlMs = 0;
    lastLogMs = 0;
    lastLineError = 0;
    lastTurnValue = 0;
    lastIrTurnValue = 0;
    lastLineSide = 1;
    lastDerivativeValue = 0.0f;
    lastContrastValue = 0;
    robot.reset_encoder_speed_control();
}

int16_t LineFollower::clampTurn(float turn) const
{
    if (turn > config.maxTurn)
    {
        return config.maxTurn;
    }

    if (turn < -config.maxTurn)
    {
        return -config.maxTurn;
    }

    return static_cast<int16_t>(turn);
}

bool LineFollower::readLineError(int16_t& error, uint16_t& contrast)
{
    sensors.update();

    uint16_t minValue = IR_READ_TIMEOUT_US;
    uint16_t maxValue = 0;

    for (uint8_t i = 0; i < sensors.getCount(); i++)
    {
        uint16_t value = sensors.getValue(i);
        if (value < minValue)
        {
            minValue = value;
        }

        if (value > maxValue)
        {
            maxValue = value;
        }
    }

    contrast = maxValue - minValue;
    lastContrastValue = contrast;

    int32_t weightedSum = 0;
    uint32_t signalSum = 0;
    int16_t center = static_cast<int16_t>(sensors.getCount() - 1) / 2;

    for (uint8_t i = 0; i < sensors.getCount(); i++)
    {
        uint16_t value = sensors.getValue(i);
        uint16_t signal =
            value > config.blackThreshold ? value - config.blackThreshold : 0;
        int16_t position =
            (static_cast<int16_t>(i) - center) * config.sensorStep;

        weightedSum += static_cast<int32_t>(signal) * position;
        signalSum += signal;
    }

    if (signalSum == 0)
    {
        lineVisible = false;
        return false;
    }

    error = static_cast<int16_t>(
        weightedSum / static_cast<int32_t>(signalSum)
    );
    lineVisible = true;
    return true;
}

bool LineFollower::readLineError(int16_t& error)
{
    uint16_t contrast = 0;
    return readLineError(error, contrast);
}

void LineFollower::printLog(const char* mode, int16_t error, int16_t turn)
{
    if (config.logOutput == nullptr)
    {
        return;
    }

    int32_t fl = 0;
    int32_t fr = 0;
    int32_t rl = 0;
    int32_t rr = 0;
    robot.get_encoder_counts(fl, fr, rl, rr);

    config.logOutput->print(mode);
    config.logOutput->print(" e=");
    config.logOutput->print(error);
    config.logOutput->print(" turn=");
    config.logOutput->print(turn);
    config.logOutput->print(" enc=");
    config.logOutput->print(fl);
    config.logOutput->print(',');
    config.logOutput->print(fr);
    config.logOutput->print(',');
    config.logOutput->print(rl);
    config.logOutput->print(',');
    config.logOutput->print(rr);
    config.logOutput->print(" ir=");

    for (uint8_t i = 0; i < sensors.getCount(); i++)
    {
        config.logOutput->print(sensors.getValue(i));
        if (i + 1 < sensors.getCount())
        {
            config.logOutput->print(',');
        }
    }

    config.logOutput->println();
}

void LineFollower::update()
{
    update(config.baseSpeed);
}

void LineFollower::update(int16_t speed)
{
    uint32_t now = millis();
    robot.update_encoder_speed_control();

    if (
        config.requireEncoderSpeedControl &&
        !robot.encoder_speed_control_enabled()
    )
    {
        robot.stop_all();
        if (now - lastLogMs >= config.logIntervalMs)
        {
            lastLogMs = now;
            printLog("encoder_off", lastLineError, 0);
        }
        return;
    }

    if (now - lastControlMs < config.controlIntervalMs)
    {
        return;
    }

    float dt = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;

    int16_t error = 0;
    uint16_t contrast = 0;
    bool seen = readLineError(error, contrast);

    if (!seen)
    {
        haveLastError = false;
        lastIrTurnValue = 0;
        lastTurnValue =
            lastLineSide < 0 ? -config.searchTurn : config.searchTurn;
        robot.drive(0, 0, lastTurnValue);
        robot.update_encoder_speed_control();

        if (now - lastLogMs >= config.logIntervalMs)
        {
            lastLogMs = now;
            printLog("search", lastLineError, lastTurnValue);
        }
        return;
    }

    if (error > 250)
    {
        lastLineSide = 1;
    }

    if (error < -250)
    {
        lastLineSide = -1;
    }

    lastDerivativeValue = haveLastError && dt > 0.0f
        ? (error - lastLineError) / dt
        : 0.0f;

    if (abs(error) >= config.hardTurnError)
    {
        lastIrTurnValue = 0;
        lastTurnValue =
            error < 0 ? -config.hardTurnSpeed : config.hardTurnSpeed;
        robot.drive(config.hardTurnForward, 0, lastTurnValue);
        robot.update_encoder_speed_control();
        lastLineError = error;
        haveLastError = true;

        if (now - lastLogMs >= config.logIntervalMs)
        {
            lastLogMs = now;
            printLog("hard", error, lastTurnValue);
        }
        return;
    }

    lastIrTurnValue = clampTurn(
        config.irDirection
        * (config.irKp * error + config.irKd * lastDerivativeValue)
    );
    lastTurnValue = lastIrTurnValue;

    robot.drive(speed, 0, lastTurnValue);
    robot.update_encoder_speed_control();
    lastLineError = error;
    haveLastError = true;

    if (now - lastLogMs >= config.logIntervalMs)
    {
        lastLogMs = now;
        printLog("line", error, lastTurnValue);
    }
}

void LineFollower::setLogOutput(Stream* output)
{
    config.logOutput = output;
}

void LineFollower::setIrKp(float kp)
{
    config.irKp = kp;
    haveLastError = false;
    lastDerivativeValue = 0.0f;
}

void LineFollower::setIrKd(float kd)
{
    config.irKd = kd;
    haveLastError = false;
    lastDerivativeValue = 0.0f;
}

void LineFollower::setIrPd(float kp, float kd)
{
    config.irKp = kp;
    config.irKd = kd;
    haveLastError = false;
    lastDerivativeValue = 0.0f;
}

bool LineFollower::hasLine() const
{
    return lineVisible;
}

float LineFollower::irKp() const
{
    return config.irKp;
}

float LineFollower::irKd() const
{
    return config.irKd;
}

int16_t LineFollower::lastError() const
{
    return lastLineError;
}

int16_t LineFollower::lastTurn() const
{
    return lastTurnValue;
}

int16_t LineFollower::lastIrTurn() const
{
    return lastIrTurnValue;
}

int16_t LineFollower::lastEncoderTurn() const
{
    return 0;
}

float LineFollower::lastDerivative() const
{
    return lastDerivativeValue;
}

uint16_t LineFollower::lastContrast() const
{
    return lastContrastValue;
}

void LineFollower::getEncoderCounts(
    int32_t& fl,
    int32_t& fr,
    int32_t& rl,
    int32_t& rr
) const
{
    robot.get_encoder_counts(fl, fr, rl, rr);
}
