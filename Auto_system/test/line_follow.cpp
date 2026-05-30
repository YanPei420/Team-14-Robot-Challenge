#include <Arduino.h>
#include <Encoder.h>

#include "IRConfig.h"
#include "IRSensor.h"
#include "MotorConfig.h"
#include "MotoronDrive.h"

constexpr int16_t BASE_SPEED = 220;
constexpr int16_t SEARCH_TURN = 160;
constexpr int16_t MAX_TURN = 450;
constexpr uint16_t MIN_CONTRAST = 50;
constexpr uint32_t CONTROL_MS = 30;
constexpr uint32_t LOG_MS = 250;

constexpr float IR_KP = 0.08f;
constexpr float IR_KD = 0.003f;
constexpr float IR_DIR = -1.0f;
constexpr float ENCODER_KP = 2.0f;
constexpr int16_t SENSOR_STEP = 1000;

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
IRSensor ir(IR_PINS, IR_SENSOR_COUNT);

Encoder encoderFL(MOTOR_ENCODER_FRONT_LEFT_A_PIN, MOTOR_ENCODER_FRONT_LEFT_B_PIN);
Encoder encoderFR(MOTOR_ENCODER_FRONT_RIGHT_A_PIN, MOTOR_ENCODER_FRONT_RIGHT_B_PIN);
Encoder encoderRL(MOTOR_ENCODER_REAR_LEFT_A_PIN, MOTOR_ENCODER_REAR_LEFT_B_PIN);
Encoder encoderRR(MOTOR_ENCODER_REAR_RIGHT_A_PIN, MOTOR_ENCODER_REAR_RIGHT_B_PIN);

long lastFL = 0;
long lastFR = 0;
long lastRL = 0;
long lastRR = 0;

uint32_t lastControlMs = 0;
uint32_t lastLogMs = 0;
int16_t lastError = 0;
int16_t lastTurn = 0;
float lastDerivative = 0.0f;
bool haveLastError = false;
bool motorReady = false;

int16_t clampTurn(float turn)
{
    if (turn > MAX_TURN) return MAX_TURN;
    if (turn < -MAX_TURN) return -MAX_TURN;
    return static_cast<int16_t>(turn);
}

bool readLine(int16_t& error, uint16_t& contrast)
{
    ir.update();

    uint16_t minValue = IR_READ_TIMEOUT_US;
    uint16_t maxValue = 0;

    for (uint8_t i = 0; i < ir.getCount(); i++)
    {
        uint16_t value = ir.getValue(i);
        if (value < minValue) minValue = value;
        if (value > maxValue) maxValue = value;
    }

    contrast = maxValue - minValue;
    if (contrast < MIN_CONTRAST) return false;

    int32_t weightedSum = 0;
    uint32_t signalSum = 0;
    int16_t center = static_cast<int16_t>(ir.getCount() - 1) / 2;

    for (uint8_t i = 0; i < ir.getCount(); i++)
    {
        uint16_t signal = ir.getValue(i) - minValue;
        int16_t position = (static_cast<int16_t>(i) - center) * SENSOR_STEP;
        weightedSum += static_cast<int32_t>(signal) * position;
        signalSum += signal;
    }

    if (signalSum == 0) return false;
    error = static_cast<int16_t>(weightedSum / static_cast<int32_t>(signalSum));
    return true;
}

int16_t encoderBalanceTurn(long fl, long fr, long rl, long rr)
{
    long dFL = abs(fl - lastFL);
    long dFR = abs(fr - lastFR);
    long dRL = abs(rl - lastRL);
    long dRR = abs(rr - lastRR);

    lastFL = fl;
    lastFR = fr;
    lastRL = rl;
    lastRR = rr;

    long leftTicks = dFL + dRL;
    long rightTicks = dFR + dRR;
    return clampTurn((leftTicks - rightTicks) * ENCODER_KP);
}

void updateLineFollow()
{
    uint32_t now = millis();
    if (now - lastControlMs < CONTROL_MS) return;

    float dt = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;

    long fl = encoderFL.read();
    long fr = encoderFR.read();
    long rl = encoderRL.read();
    long rr = encoderRR.read();

    int16_t error = 0;
    uint16_t contrast = 0;
    bool seen = readLine(error, contrast);

    if (!motorReady)
    {
        robot.stop_all();
        return;
    }

    if (!seen)
    {
        haveLastError = false;
        lastTurn = lastTurn < 0 ? -SEARCH_TURN : SEARCH_TURN;
        robot.drive(0, 0, lastTurn);
        return;
    }

    lastDerivative = haveLastError && dt > 0.0f
        ? (error - lastError) / dt
        : 0.0f;

    int16_t irTurn = clampTurn(IR_DIR * (IR_KP * error + IR_KD * lastDerivative));
    int16_t encTurn = encoderBalanceTurn(fl, fr, rl, rr);
    lastTurn = clampTurn(irTurn + encTurn);

    robot.drive(BASE_SPEED, 0, lastTurn);
    lastError = error;
    haveLastError = true;

    if (now - lastLogMs >= LOG_MS)
    {
        lastLogMs = now;
        Serial.print("e=");
        Serial.print(error);
        Serial.print(" turn=");
        Serial.print(lastTurn);
        Serial.print(" enc=");
        Serial.print(fl);
        Serial.print(',');
        Serial.print(fr);
        Serial.print(',');
        Serial.print(rl);
        Serial.print(',');
        Serial.println(rr);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}

    ir.begin();
    encoderFL.write(0);
    encoderFR.write(0);
    encoderRL.write(0);
    encoderRR.write(0);

    motorReady = robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);
    robot.clear_status_flags();
    robot.stop_all();

    Serial.println(motorReady ? "line_follow_encoder_start" : "motor=FAILED");
}

void loop()
{
    updateLineFollow();
}
