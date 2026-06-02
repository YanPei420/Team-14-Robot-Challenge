#include <Arduino.h>

#include "MotorConfig.h"
#include "MotoronDrive.h"

namespace
{
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t LOG_INTERVAL_MS = 100;

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);

int16_t testSpeed = MOTOR_DISTANCE_MOVE_DEFAULT_SPEED;
bool testRunning = false;
char activeMove = '-';
uint32_t lastLogMs = 0;

int32_t lastFrontLeft = 0;
int32_t lastFrontRight = 0;
int32_t lastRearLeft = 0;
int32_t lastRearRight = 0;

int32_t absCount(int32_t value)
{
    return value < 0 ? -value : value;
}

void sampleCounts()
{
    robot.get_encoder_counts(
        lastFrontLeft,
        lastFrontRight,
        lastRearLeft,
        lastRearRight
    );
}

int32_t averageAbsCounts()
{
    return (
        absCount(lastFrontLeft) +
        absCount(lastFrontRight) +
        absCount(lastRearLeft) +
        absCount(lastRearRight)
    ) / 4;
}

void printCounts(const char* prefix)
{
    Serial.print(prefix);
    Serial.print(" FL=");
    Serial.print(lastFrontLeft);
    Serial.print(" FR=");
    Serial.print(lastFrontRight);
    Serial.print(" RL=");
    Serial.print(lastRearLeft);
    Serial.print(" RR=");
    Serial.print(lastRearRight);
    Serial.print(" avg_abs=");
    Serial.println(averageAbsCounts());
}

void printHelp()
{
    Serial.println();
    Serial.println("Encoder 25cm distance move test");
    Serial.println("Commands:");
    Serial.println("  F - forward 25cm");
    Serial.println("  B - backward 25cm");
    Serial.println("  L - left 25cm");
    Serial.println("  R - right 25cm");
    Serial.println("  2 - speed 220");
    Serial.println("  3 - speed 300");
    Serial.println("  6 - speed 600");
    Serial.println("  + - speed +20");
    Serial.println("  - - speed -20");
    Serial.println("  C - print current encoder counts");
    Serial.println("  S - stop/cancel");
    Serial.println("  H or ? - help");
    Serial.print("Expected 25cm counts: ");
    Serial.println(robot.distance_cm_to_encoder_counts(25.0f));
    Serial.print("Current speed: ");
    Serial.println(testSpeed);
    Serial.println();
}

void printSpeed()
{
    Serial.print("[speed] ");
    Serial.println(testSpeed);
}

void setSpeed(int16_t speed)
{
    if (speed < MOTOR_DISTANCE_MOVE_MIN_SPEED)
    {
        speed = MOTOR_DISTANCE_MOVE_MIN_SPEED;
    }

    if (speed > MOTOR_MAX_SPEED)
    {
        speed = MOTOR_MAX_SPEED;
    }

    testSpeed = speed;
    printSpeed();
}

void startMove(char command)
{
    if (robot.distance_move_active())
    {
        Serial.println("[move] already running; press S to cancel first");
        return;
    }

    bool started = false;

    switch (command)
    {
        case 'F':
            started = robot.forward_25cm(testSpeed);
            break;

        case 'B':
            started = robot.backward_25cm(testSpeed);
            break;

        case 'L':
            started = robot.left_25cm(testSpeed);
            break;

        case 'R':
            started = robot.right_25cm(testSpeed);
            break;

        default:
            return;
    }

    if (!started)
    {
        Serial.println("[move] failed to start encoder distance move");
        return;
    }

    testRunning = true;
    activeMove = command;
    lastLogMs = 0;

    Serial.print("[move] start ");
    Serial.print(activeMove);
    Serial.print(" speed=");
    Serial.print(testSpeed);
    Serial.print(" target_counts=");
    Serial.println(robot.distance_cm_to_encoder_counts(25.0f));
}

void handleCommand(char command)
{
    if (command >= 'a' && command <= 'z')
    {
        command = command - 'a' + 'A';
    }

    switch (command)
    {
        case 'F':
        case 'B':
        case 'L':
        case 'R':
            startMove(command);
            break;

        case '2':
            setSpeed(220);
            break;

        case '3':
            setSpeed(300);
            break;

        case '6':
            setSpeed(600);
            break;

        case '+':
            setSpeed(testSpeed + 20);
            break;

        case '-':
            setSpeed(testSpeed - 20);
            break;

        case 'C':
            sampleCounts();
            printCounts("[counts]");
            break;

        case 'S':
            robot.cancel_distance_move();
            testRunning = false;
            activeMove = '-';
            Serial.println("[move] cancelled");
            break;

        case 'H':
        case '?':
            printHelp();
            break;

        case '\r':
        case '\n':
            break;

        default:
            Serial.println("[serial] unknown command; press H for help");
            break;
    }
}

void pollSerial()
{
    while (Serial.available() > 0)
    {
        handleCommand(static_cast<char>(Serial.read()));
    }
}

void updateMove()
{
    if (!robot.distance_move_active())
    {
        robot.update();
        return;
    }

    sampleCounts();

    const uint32_t now = millis();
    if (now - lastLogMs >= LOG_INTERVAL_MS)
    {
        lastLogMs = now;
        printCounts("[moving]");
    }

    const bool completed = robot.update_distance_move();

    if (testRunning && !robot.distance_move_active())
    {
        Serial.print("[move] ");
        Serial.print(activeMove);
        Serial.print(completed && robot.distance_move_complete()
            ? " complete"
            : " stopped/timeout");
        Serial.print(" expected=");
        Serial.print(robot.distance_cm_to_encoder_counts(25.0f));
        printCounts(" final_before_stop");

        testRunning = false;
        activeMove = '-';
    }
}
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial)
    {
        ;
    }

    robot.begin();
    robot.set_max_speed(MOTOR_MAX_SPEED);

    if (!robot.encoder_speed_control_ready())
    {
        Serial.println("[init] encoder speed control failed");
    }
    else
    {
        Serial.println("[init] encoder speed control ready");
    }

    robot.stop_all();
    printHelp();
}

void loop()
{
    pollSerial();
    updateMove();
    delay(5);
}
