#include <Wire.h>
#include <Motoron.h>

MotoronI2C mc;

const int encoderPinA = A1; 
const int encoderPinB = A3; 
volatile long encoderPos = 0; 

long currentTarget = 2000; 
unsigned long lastPrintTime = 0;
int activeCommandedSpeed = 0; 

long lastEncoderPosForStall = 0;
unsigned long lastStallCheckTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting The Final Master Code with Stall Protection...");

  Wire.begin();
  mc.reinitialize();
  mc.clearResetFlag();
  
  mc.setCommandTimeoutMilliseconds(1000); 
  
  mc.setMaxAcceleration(1, 200);
  mc.setMaxDeceleration(1, 300);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, CHANGE);
}

void loop() {
  mc.resetCommandTimeout();

  bool hasArrived = moveToPositionSimple(currentTarget);

  // Every 500 milliseconds, check if the clamp is jammed
  /*
  if (millis() - lastStallCheckTime > 500) {
    
    // Are we commanding heavy power? AND has the encoder barely moved?
    long pulsesMoved = abs(encoderPos - lastEncoderPosForStall);
    /*
    if (abs(activeCommandedSpeed) >= 600 && pulsesMoved <= 5) {
      // 🚨 EMERGENCY SHUTDOWN TRIGGERED 🚨
      mc.setSpeed(1, 0); 
      Serial.println("\n=========================================");
      Serial.println("🚨 CRITICAL ERROR: CLAMP STALLED OR JAMMED! 🚨");
      Serial.println("EMERGENCY BRAKE APPLIED. SYSTEM LOCKED.");
      Serial.println("=========================================\n");
      
      // Trap the Arduino in an infinite loop so it can't restart the motor
      while(true) { 
        delay(10); 
      } 
    }
    
    
    // Update stall memory for the next 500ms check
    lastEncoderPosForStall = encoderPos;
    lastStallCheckTime = millis();
  }
  */

  if (millis() - lastPrintTime >= 100) {
    Serial.print("Target: ");
    Serial.print(currentTarget);
    Serial.print("  |  Live Position: ");
    Serial.println(encoderPos);
    lastPrintTime = millis();
  }

  if (hasArrived) {
    Serial.println("\n--- TARGET REACHED! Braking for 3 seconds ---");
    delay(3000); 
    
    if (currentTarget == 2000) {
      currentTarget = 0;
    } else {
      currentTarget = 2000;
    }
  }
}

bool moveToPositionSimple(long targetPos) {
  long error = targetPos - encoderPos;
  int tolerance = 15; 
  int crawlSpeed = -800; // Full power to break friction!
  int desiredSpeed = 0; 

  if (abs(error) <= tolerance) {
    desiredSpeed = 0; 
  } else if (error > 0) {
    desiredSpeed = crawlSpeed;  
  } else {
    desiredSpeed = -crawlSpeed; 
  }

  if (desiredSpeed != activeCommandedSpeed) {
    mc.setSpeed(1, desiredSpeed); 
    activeCommandedSpeed = desiredSpeed; // Update the global variable for the stall detector  
  }

  if (desiredSpeed == 0) return true; 
  return false; 
}

void readEncoder() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderPos--; 
  } else {
    encoderPos++; 
  }
}