const int buttonPin = 17;

const int redPin = 18;
const int greenPin = 19;

unsigned long lastFlash = 0;
bool redState = false;

bool deadSignal = false;

unsigned long greenStart = 0;
bool greenActive = false;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // internal pull-up resistor

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
}

void loop() {
  int buttonState = digitalRead(buttonPin);
  
  if (deadSignal){
    if (millis() - lastFlash >= 300){
      lastFlash = millis();
      redState = !redState;
      digitalWrite(redPin, redState);
    }
  	digitalWrite(greenPin, LOW);
    return;
  }
  
  if (greenActive) {
    if (millis() - greenStart < 3000) {
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
      return;
    } else {
      greenActive = false;
    }
  }
  
  if (buttonState == LOW) {
    // Button pressed
    greenActive = true;
    greenStart = millis();
    
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
    return;
  }
  
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, LOW);
}
