#include <Arduino.h>
#include <Wire.h>

#include <MFRC522DriverI2C.h>
#include <MFRC522v2.h>

MFRC522DriverI2C driver(0x28);
MFRC522 rfid(driver);

void setup()
{
    Serial.begin(115200);

    while (!Serial);

    Wire.begin();

    rfid.PCD_Init();

    Serial.println("Hold an RFID card near the reader...");
}

void loop()
{
    if (!rfid.PICC_IsNewCardPresent())
    {
        delay(50);
        return;
    }

    if (!rfid.PICC_ReadCardSerial())
    {
        delay(50);
        return;
    }

    Serial.print("Card UID: ");

    for (byte i = 0; i < rfid.uid.size; i++)
    {
        if (rfid.uid.uidByte[i] < 0x10)
        {
            Serial.print("0");
        }

        Serial.print(
            rfid.uid.uidByte[i],
            HEX
        );

        Serial.print(" ");
    }

    Serial.println();

    rfid.PICC_HaltA();

    delay(500);
}