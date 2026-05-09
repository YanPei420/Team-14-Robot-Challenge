const int buttonPin = 17;

const int redPin = 18;
const int greenPin = 19;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
}

void loop() {
  int buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    // Button pressed
    digitalWrite(redPin, LOW);   // turn ON red
    digitalWrite(greenPin, HIGH); // turn OFF green
  } else {
    // Button not pressed
    digitalWrite(redPin, HIGH);  // turn OFF red
    digitalWrite(greenPin, LOW); // turn ON green
  }
}
