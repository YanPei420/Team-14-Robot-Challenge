#include <Modulino.h>
ModulinoPixels pixels;
int brightness = 25;
const int buttonPin = 2;

void setup() {
  // put your setup code here, to run once:
  Modulino.begin();
  pixels.begin();
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly
  int buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH){
    for (int i = 0; i < 8; i++){
      pixels.set(i,RED, brightness);
      pixels.show();
    }
  } else{
    for (int i = 0; i < 8; i++){
      pixels.set(i,GREEN, brightness);
      pixels.show();
    }
    delay(4000);
  }
}
