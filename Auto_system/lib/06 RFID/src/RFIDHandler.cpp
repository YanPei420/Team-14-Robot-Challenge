#include "RFIDHandler.h"

RFIDHandler::RFIDHandler(
    uint8_t readerI2CAddress
)
    : i2cAddress(readerI2CAddress),
      driver(readerI2CAddress),
      rfid(driver)
{
    cardPresent = false;
    uidSize = 0;
    clearUID();
}

void RFIDHandler::begin()
{
    Wire.begin();
    rfid.PCD_Init();

    Serial.print("RFID reader initialized at I2C 0x");
    Serial.println(i2cAddress, HEX);
}

bool RFIDHandler::update()
{
    if (!rfid.PICC_IsNewCardPresent())
    {
        cardPresent = false;
        return false;
    }

    if (!rfid.PICC_ReadCardSerial())
    {
        cardPresent = false;
        return false;
    }

    clearUID();

    uidSize = rfid.uid.size;

    for (byte i = 0; i < uidSize && i < sizeof(uidBytes); ++i)
    {
        uidBytes[i] = rfid.uid.uidByte[i];
    }

    cardPresent = true;

    rfid.PICC_HaltA();

    return true;
}

bool RFIDHandler::isCardPresent() const
{
    return cardPresent;
}

byte RFIDHandler::getUIDSize() const
{
    return uidSize;
}

const byte* RFIDHandler::getUIDBytes() const
{
    return uidBytes;
}

String RFIDHandler::getUID() const
{
    String uidString;

    for (byte i = 0; i < uidSize; ++i)
    {
        if (uidBytes[i] < 0x10)
        {
            uidString += "0";
        }

        uidString += String(uidBytes[i], HEX);

        if (i + 1 < uidSize)
        {
            uidString += " ";
        }
    }

    uidString.toUpperCase();
    return uidString;
}

void RFIDHandler::clearUID()
{
    for (byte i = 0; i < sizeof(uidBytes); ++i)
    {
        uidBytes[i] = 0;
    }

    uidSize = 0;
}
