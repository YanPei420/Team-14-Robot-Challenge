// HC-SR04 basic distance test
const uint8_t ECHO_PIN = 11;
const uint8_t TRIG_PIN = 12;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.println("HC-SR04 Distance Test");
  Serial.println("---------------------");
}

void loop() {
  // Send 10us trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo duration (timeout = 30ms, ~5m max range)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  // Convert to cm: speed of sound = 343 m/s = 0.0343 cm/us, divide by 2 for round trip
  float distance_cm = (duration * 0.0343) / 2.0;

  if (duration == 0) {
    Serial.println("ERROR: No echo - check wiring");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance_cm, 1); // 1 decimal place
    Serial.println(" cm");
  }

  delay(200);
}