#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "LineFollower.h"

constexpr size_t TUNE_COMMAND_BUFFER_SIZE = 64;

MotoronDrive robot(MOTORON_ADDR_FRONT, MOTORON_ADDR_REAR);
IRSensor ir(IR_PINS, IR_SENSOR_COUNT);
LineFollower lineFollower(robot, ir);

bool motorReady = false;
bool encoderControlReady = false;
char tuneCommand[TUNE_COMMAND_BUFFER_SIZE] = {0};
size_t tuneCommandLength = 0;

char lowerAscii(char value)
{
    if (value >= 'A' && value <= 'Z')
    {
        return static_cast<char>(value - 'A' + 'a');
    }

    return value;
}

bool isSeparator(char value)
{
    return value == '\0' ||
        value == ' ' ||
        value == '\t' ||
        value == '=' ||
        value == ':' ||
        value == ',';
}

char* trimText(char* text)
{
    while (*text == ' ' || *text == '\t' || *text == '!')
    {
        text++;
    }

    char* end = text + strlen(text);
    while (end > text)
    {
        const char previous = *(end - 1);
        if (previous != ' ' && previous != '\t')
        {
            break;
        }

        end--;
    }
    *end = '\0';

    return text;
}

bool commandMatches(const char* text, const char* command)
{
    while (*command != '\0')
    {
        if (lowerAscii(*text) != *command)
        {
            return false;
        }

        text++;
        command++;
    }

    return isSeparator(*text);
}

bool parseFloatValue(char*& text, float& value)
{
    while (isSeparator(*text) && *text != '\0')
    {
        text++;
    }

    if (*text == '\0')
    {
        return false;
    }

    char* end = nullptr;
    value = strtof(text, &end);
    if (end == text)
    {
        return false;
    }

    text = end;
    return true;
}

bool validTuneValue(float value)
{
    return value >= 0.0f && value <= 10.0f;
}

void printTuneValues()
{
    Serial.print("[line] kp=");
    Serial.print(lineFollower.irKp(), 6);
    Serial.print(" kd=");
    Serial.println(lineFollower.irKd(), 6);
}

void printTuneHelp()
{
    Serial.println("Line follow PD tune commands:");
    Serial.println("  kp=0.08");
    Serial.println("  kd=0.004");
    Serial.println("  pd 0.07 0.004");
    Serial.println("  show");
    Serial.println("Commands are applied after newline.");
}

void handleTuneCommand(char* rawCommand)
{
    char* command = trimText(rawCommand);
    if (*command == '\0')
    {
        return;
    }

    if (commandMatches(command, "?") || commandMatches(command, "help"))
    {
        printTuneHelp();
        printTuneValues();
        return;
    }

    if (commandMatches(command, "show"))
    {
        printTuneValues();
        return;
    }

    if (commandMatches(command, "kp"))
    {
        char* valueText = command + 2;
        float kp = 0.0f;
        if (!parseFloatValue(valueText, kp) || !validTuneValue(kp))
        {
            Serial.println("[line] invalid kp, expected 0..10");
            return;
        }

        lineFollower.setIrKp(kp);
        printTuneValues();
        return;
    }

    if (commandMatches(command, "kd"))
    {
        char* valueText = command + 2;
        float kd = 0.0f;
        if (!parseFloatValue(valueText, kd) || !validTuneValue(kd))
        {
            Serial.println("[line] invalid kd, expected 0..10");
            return;
        }

        lineFollower.setIrKd(kd);
        printTuneValues();
        return;
    }

    if (commandMatches(command, "pd"))
    {
        char* valueText = command + 2;
        float kp = 0.0f;
        float kd = 0.0f;
        if (!parseFloatValue(valueText, kp) ||
            !parseFloatValue(valueText, kd) ||
            !validTuneValue(kp) ||
            !validTuneValue(kd))
        {
            Serial.println("[line] invalid pd, expected: pd <kp> <kd>");
            return;
        }

        lineFollower.setIrPd(kp, kd);
        printTuneValues();
        return;
    }

    Serial.println("[line] unknown command; send help");
}

void pollTuneSerial()
{
    while (Serial.available() > 0)
    {
        const char input = static_cast<char>(Serial.read());
        if (input == '\r' || input == '\n')
        {
            if (tuneCommandLength > 0)
            {
                tuneCommand[tuneCommandLength] = '\0';
                handleTuneCommand(tuneCommand);
                tuneCommandLength = 0;
                tuneCommand[0] = '\0';
            }
            continue;
        }

        if (tuneCommandLength + 1 >= TUNE_COMMAND_BUFFER_SIZE)
        {
            tuneCommandLength = 0;
            tuneCommand[0] = '\0';
            Serial.println("[line] command too long");
            continue;
        }

        tuneCommand[tuneCommandLength] = input;
        tuneCommandLength++;
        tuneCommand[tuneCommandLength] = '\0';
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}

    ir.begin();

    motorReady = robot.begin();
    if (motorReady)
    {
        robot.set_max_speed(MOTOR_MAX_SPEED);
        robot.clear_status_flags();
        robot.stop_all();
        encoderControlReady = robot.begin_encoder_speed_control();
        lineFollower.reset();
    }

    lineFollower.setLogOutput(&Serial);

    Serial.print("motor=");
    Serial.print(motorReady ? "OK" : "FAILED");
    Serial.print(" encoder_control=");
    Serial.println(encoderControlReady ? "OK" : "FAILED");
    printTuneHelp();
    printTuneValues();
}

void loop()
{
    pollTuneSerial();

    if (!motorReady || !encoderControlReady)
    {
        robot.stop_all();
        return;
    }

    lineFollower.update();
}
