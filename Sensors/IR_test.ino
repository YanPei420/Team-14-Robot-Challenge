const uint8_t SensorCount = 9;
const uint8_t sensorPins[] = {45 ,46, 47, 48, 49, 50, 51, 52, 53};

void setup() {
  delay(5000);

  Serial.begin(9600);
  Serial.println("SYSTEM BOOT: Library-Bypass Diagnostic Started");

  Serial.println("Emitters ON. Reading pins directly. Wave white paper.");
  Serial.println("P45\tP46\tP47\tP48\tP49\tP50\tP51\tP52\tP53");
  Serial.println("-------------------------------------------------------------------------");
}

void loop() {
  uint16_t sensorValues[SensorCount];

  // 1. Charge the capacitors on the sensor board
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  // Wait 15 microseconds for capacitors to fully charge
  delayMicroseconds(15);

  // 2. Switch pins to INPUT to let them discharge
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorValues[i] = 1000; // Default to max timeout of 1000
  }

  // 3. Measure discharge time WITHOUT disabling system interrupts
  unsigned long startTime = micros();
  while (micros() - startTime < 1000) { // 1000us absolute max timeout
    for (uint8_t i = 0; i < SensorCount; i++) {
      // If the pin drops to LOW, record how long it took
      if (sensorValues[i] == 1000 && digitalRead(sensorPins[i]) == LOW) {
        sensorValues[i] = micros() - startTime;
      }
    }
  }

  // 4. Print the raw results
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  // Yield to Mbed OS so it doesn't crash the native USB
  delay(250);
}
