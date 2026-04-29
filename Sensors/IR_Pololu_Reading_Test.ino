#include <QTRSensors.h>

QTRSensors qtr;

// Reading sensors 4, 5, 6
const uint8_t SensorCount = 3;
uint16_t sensorValues[SensorCount];

// Connect sensor 4 to A0, sensor 5 to A1, sensor 6 to A2
const uint8_t qtrPins[SensorCount] = {A0, A1, A2};

void setup() {
  Serial.begin(115200);

  // Wait for Serial to initialise on Giga R1
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000);

  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);

  delay(500);

  Serial.println("QTR-HD-09RC sensor test - sensors 4, 5, 6");
}

void loop() {
  qtr.read(sensorValues);

  // Print values for sensor 4, 5, 6
  Serial.print("S4: "); Serial.print(sensorValues[0]);
  Serial.print("  S5: "); Serial.print(sensorValues[1]);
  Serial.print("  S6: "); Serial.println(sensorValues[2]);

  delay(100);
}