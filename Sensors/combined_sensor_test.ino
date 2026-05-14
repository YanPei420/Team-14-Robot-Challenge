#include <Wire.h>
#include "MFRC522_I2C.h"


// ---------------- RFID (MFRC522 over I2C) ----------------
MFRC522_I2C rfid(0x28, 255);

// ---------------- IR sensor array ----------------
const uint8_t SensorCount = 9;
const uint8_t sensorPins[] = {45, 46, 47, 48, 49, 50, 51, 52, 53};
uint16_t sensorValues[SensorCount];  // global: accessible to other functions if needed

// ---------------- LiDAR latest reading (updated continuously) ----------------
int  lidarDist = 0;       // most recent valid distance in cm
int  lidarAmp  = 0;       // most recent signal amplitude
bool lidarValid = false;  // true if a valid frame has been received

// ---------------- Non-blocking timing ----------------
// Both IR and LiDAR PRINT at this same interval so their output rates match
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 100;   // ms between prints (10 Hz for both)

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // RFID init
  Wire.begin();
  rfid.PCD_Init();

  // LiDAR init (Serial1 = pin 18 TX / pin 19 RX on GIGA R1)
  Serial1.begin(115200);

  Serial.println("=== SYSTEM BOOT ===");
  Serial.println("TF-Luna | IR Array | RFID Reader - all systems initialised");
  Serial.println("Hold an RFID card near the reader...");
  Serial.println("---------------------------------------------------");
}

void loop() {
  handleRFID();
  updateLiDAR();   // continuously drains the serial buffer (high frequency)
  updateIR();      // continuously reads IR sensors (high frequency)
  printSensors();  // prints BOTH at the same fixed interval
}

// ---------------- RFID ----------------
void handleRFID() {
  // Non-blocking: just return if no new card
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

  Serial.print("[RFID]  Card UID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  rfid.PICC_HaltA();
}

// ---------------- LiDAR (TF-Luna) ----------------
// Drains ALL available frames every loop, keeping only the latest reading.
// This prevents the serial buffer from overflowing and keeps data fresh.
void updateLiDAR() {
  // Process every complete frame currently in the buffer
  while (Serial1.available() >= 9) {
    // Sync to frame header: discard bytes until we see 0x59 0x59
    if (Serial1.read() != 0x59) {
      continue;  // not aligned, drop this byte and keep scanning
    }
    if (Serial1.peek() != 0x59) {
      continue;  // single 0x59, not a real header
    }
    Serial1.read();  // consume the second 0x59

    uint8_t raw[7];
    Serial1.readBytes(raw, 7);  // remaining 7 bytes of the frame

    // Verify checksum: low 8 bits of sum of first 8 bytes
    uint8_t checksum = 0x59 + 0x59;
    for (int i = 0; i < 6; i++) checksum += raw[i];
    if (checksum != raw[6]) {
      continue;  // corrupted frame, skip and keep scanning
    }

    // Valid frame - update the latest reading
    lidarDist  = raw[0] + raw[1] * 256;
    lidarAmp   = raw[2] + raw[3] * 256;
    lidarValid = true;
  }
}

// IR sensor array
void updateIR() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(15);

  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorValues[i] = 1000; 
  }

  unsigned long startTime = micros();
  while (micros() - startTime < 1000) {  
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] == 1000 && digitalRead(sensorPins[i]) == LOW) {
        sensorValues[i] = micros() - startTime;
      }
    }
  }
}

void printSensors() {
  if (millis() - lastPrint < PRINT_INTERVAL) {
    return;
  }
  lastPrint = millis();

  // LiDAR
  Serial.print("[LiDAR] ");
  if (!lidarValid || lidarAmp < 100) {
    Serial.println("Distance: UNRELIABLE (signal too weak)");
  } else {
    Serial.print("Dist: ");
    Serial.print(lidarDist);
    Serial.println(" cm");
  }

  // IR
  Serial.print("[IR]    ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    if (i < SensorCount - 1) Serial.print('\t');
  }
  Serial.println();
}