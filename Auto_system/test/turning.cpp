#include <Arduino.h>
#include <Wire.h>

#include <MFRC522DriverI2C.h>
#include <MFRC522v2.h>

MFRC522DriverI2C driver(0x28, Wire1);
MFRC522 mfrc522(driver);

void setup()
{
    Serial.begin(115200);

    Wire1.begin();

    mfrc522.PCD_Init();

    Serial.println("RFID READY");
}

void loop()
{
    if (!mfrc522.PICC_IsNewCardPresent())
    {
        delay(50);
        return;
    }

    if (!mfrc522.PICC_ReadCardSerial())
    {
        delay(50);
        return;
    }

    Serial.print("UID: ");

    for (byte i = 0; i < mfrc522.uid.size; i++)
    {
        Serial.print(
            mfrc522.uid.uidByte[i],
            HEX
        );

        Serial.print(" ");
    }

    Serial.println();

    mfrc522.PICC_HaltA();

    delay(500);
}
