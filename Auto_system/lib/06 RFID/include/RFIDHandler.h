#ifndef RFID_HANDLER_H
#define RFID_HANDLER_H

#include <Arduino.h>
#include <Wire.h>

#include <MFRC522DriverI2C.h>
#include <MFRC522v2.h>

#include "RFIDConfig.h"

class RFIDHandler
{
private:
    uint8_t i2cAddress;

    MFRC522DriverI2C driver;
    MFRC522 rfid;

    bool cardPresent;
    byte uidBytes[10];
    byte uidSize;

    void clearUID();

public:
    RFIDHandler(uint8_t readerI2CAddress = RFID_I2C_ADDRESS);

    void begin();
    bool update();

    bool isCardPresent() const;
    byte getUIDSize() const;
    const byte* getUIDBytes() const;
    String getUID() const;
};

#endif
