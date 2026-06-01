#include <Arduino.h>

#include "LidarConfig.h"

typedef struct
{
    int distance;
    int strength;
    int temp;
    bool receiveComplete;

    uint8_t rx[9];
    uint8_t rxIndex;

    int availableBeforeRead;
    uint8_t lastByte;
    bool hasLastByte;

    uint32_t bytesRead;
    uint32_t validFrameCount;
    uint32_t checksumErrorCount;
    uint32_t droppedByteCount;
} TF;

TF lidarLeft = {0, 0, 0, false, {0}, 0, 0, 0, false, 0, 0, 0, 0};
TF lidarRight = {0, 0, 0, false, {0}, 0, 0, 0, false, 0, 0, 0, 0};

uint32_t lastPrintMs = 0;

void getLidarData(HardwareSerial& lidarSerial, TF* lidar)
{
    lidar->availableBeforeRead = lidarSerial.available();

    while (lidarSerial.available() > 0)
    {
        const int data = lidarSerial.read();
        if (data < 0)
        {
            return;
        }

        lidar->rx[lidar->rxIndex] = (uint8_t)data;
        lidar->lastByte = (uint8_t)data;
        lidar->hasLastByte = true;
        lidar->bytesRead++;

        if (lidar->rx[0] != 0x59)
        {
            lidar->rxIndex = 0;
            lidar->droppedByteCount++;
        }
        else if (lidar->rxIndex == 1 && lidar->rx[1] != 0x59)
        {
            lidar->rxIndex = 0;
            lidar->droppedByteCount++;
        }
        else if (lidar->rxIndex == 8)
        {
            int checksum = 0;
            for (uint8_t i = 0; i < 8; i++)
            {
                checksum += lidar->rx[i];
            }

            if (lidar->rx[8] == checksum % 256)
            {
                lidar->distance = lidar->rx[2] + lidar->rx[3] * 256;
                lidar->strength = lidar->rx[4] + lidar->rx[5] * 256;
                lidar->temp = (lidar->rx[6] + lidar->rx[7] * 256) / 8 - 256;
                lidar->receiveComplete = true;
                lidar->validFrameCount++;
            }
            else
            {
                lidar->checksumErrorCount++;
            }

            lidar->rxIndex = 0;
        }
        else
        {
            lidar->rxIndex++;
        }
    }
}

void printHexByte(uint8_t value)
{
    Serial.print("0x");
    if (value < 0x10)
    {
        Serial.print("0");
    }
    Serial.print(value, HEX);
}

void printLidarStatus(const char* label, TF* lidar)
{
    Serial.print(label);
    Serial.print(": ");

    if (lidar->receiveComplete)
    {
        lidar->receiveComplete = false;
        Serial.print("Distance: ");
        Serial.print(lidar->distance);
        Serial.print("cm\tStrength: ");
        Serial.print(lidar->strength);
        Serial.print("\tTemp: ");
        Serial.print(lidar->temp);
        Serial.print("C");
    }
    else
    {
        Serial.print("waiting");
    }

    Serial.print("\tavail_before=");
    Serial.print(lidar->availableBeforeRead);
    Serial.print("\tbytes=");
    Serial.print(lidar->bytesRead);
    Serial.print("\tframes=");
    Serial.print(lidar->validFrameCount);
    Serial.print("\tchecksum=");
    Serial.print(lidar->checksumErrorCount);
    Serial.print("\tdropped=");
    Serial.print(lidar->droppedByteCount);
    Serial.print("\tlast=");
    if (lidar->hasLastByte)
    {
        printHexByte(lidar->lastByte);
    }
    else
    {
        Serial.print("none");
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);

    const uint32_t serialStartMs = millis();
    while (!Serial && millis() - serialStartMs < 3000)
    {
        ;
    }

    LIDAR_SERIAL_1.begin(LIDAR_BAUD_RATE);
    LIDAR_SERIAL_2.begin(LIDAR_BAUD_RATE);

    Serial.println();
    Serial.println("TF-Luna left/right UART raw test");
    Serial.println("Left  TF-Luna TX -> LIDAR_SERIAL_1 RX");
    Serial.println("Right TF-Luna TX -> LIDAR_SERIAL_2 RX");
    Serial.println("Serial Monitor: 115200 baud");
}

void loop()
{
    getLidarData(LIDAR_SERIAL_1, &lidarLeft);
    getLidarData(LIDAR_SERIAL_2, &lidarRight);

    const uint32_t now = millis();
    if (now - lastPrintMs < LIDAR_PRINT_INTERVAL_MS)
    {
        return;
    }

    lastPrintMs = now;
    Serial.println("---");
    printLidarStatus("LEFT ", &lidarLeft);
    printLidarStatus("RIGHT", &lidarRight);
}
