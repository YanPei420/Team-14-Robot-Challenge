#define Wire Wire1
#include <Motoron.h>

// ===== MOTOR CONTROLLERS =====
// CHANGE addresses if needed!
MotoronI2C mc1(16);   // Front motors
MotoronI2C mc2(17);   // Rear motors

// ===== ENCODER PINS =====
// Front Left
#define FL_A 2
#define FL_B 4

// Front Right
#define FR_A 3
#define FR_B 5

// Rear Left
#define RL_A 18
#define RL_B 19

// Rear Right
#define RR_A 22
#define RR_B 23

// ===== ENCODER COUNTS =====
volatile long countFL = 0;
volatile long countFR = 0;
volatile long countRL = 0;
volatile long countRR = 0;

// ===== ISR FUNCTIONS =====
void ISR_FL() {
  if (digitalRead(FL_B)) countFL++;
  else countFL--;
}

void ISR_FR() {
  if (digitalRead(FR_B)) countFR++;
  else countFR--;
}

void ISR_RL() {
  if (digitalRead(RL_B)) countRL++;
  else countRL--;
}

void ISR_RR() {
  if (digitalRead(RR_B)) countRR++;
  else countRR--;
}

// ===== STOP ALL MOTORS =====
void stopMotors() {
  mc1.setSpeed(1, 0);  // FL
  mc1.setSpeed(3, 0);  // FR
  mc2.setSpeed(1, 0);  // RL
  mc2.setSpeed(3, 0);  // RR
}

// ===== DRIVE FORWARD WITH CORRECTION =====
void driveForward(int baseSpeed)
{
  // long leftAvg  = (countFL + countRL) / 2;
  // long rightAvg = (countFR + countRR) / 2;

  // long error = leftAvg - rightAvg;

  // float Kp = 1.2;   // tune this
  // int correction = Kp * error;

  // int leftSpeed  = baseSpeed - correction;
  // int rightSpeed = baseSpeed + correction;


  // leftSpeed  = constrain(leftSpeed, -800, 800);
  // rightSpeed = constrain(rightSpeed, -800, 800);

  // Front board
  mc1.setSpeed(1, baseSpeed);    // FL
  mc1.setSpeed(3, -baseSpeed);   // FR

  // Rear board
  mc2.setSpeed(1, baseSpeed);    // RR
  mc2.setSpeed(3, -baseSpeed);   // RL
}

// ===== STRAFE RIGHT =====
void strafeRight(int speed)
{
  mc1.setSpeed(1, speed);    // FR
  mc2.setSpeed(1, speed);   // FL

  mc2.setSpeed(3, -speed);    // RL
  mc1.setSpeed(3, -speed);   // RR
}

// ===== TURN LEFT =====
//void turnLeft(long targetTicks, int speed)
//{
  //countFL = countFR = countRL = countRR = 0;

  //while (abs(countFL) < targetTicks)
  //{
    //mc1.setSpeed(1, -speed);
    //mc1.setSpeed(3, speed);

    //mc2.setSpeed(1, -speed);
    //mc2.setSpeed(3, speed);
  //}

  //stopMotors();
//}

// ===== SETUP =====
void setup()
{
  Wire.begin();

  // Init BOTH Motoron boards
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();

  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();

  // Encoder pins
  pinMode(FL_A, INPUT_PULLUP);
  pinMode(FL_B, INPUT_PULLUP);

  pinMode(FR_A, INPUT_PULLUP);
  pinMode(FR_B, INPUT_PULLUP);

  pinMode(RL_A, INPUT_PULLUP);
  pinMode(RL_B, INPUT_PULLUP);

  pinMode(RR_A, INPUT_PULLUP);
  pinMode(RR_B, INPUT_PULLUP);

  // Attach interrupts (A channels only)
  attachInterrupt(digitalPinToInterrupt(FL_A), ISR_FL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_A), ISR_FR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_A), ISR_RL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_A), ISR_RR, CHANGE);
}

// ===== MAIN LOOP =====
void loop()
{
  // Reset encoders
  countFL = countFR = countRL = countRR = 0;

  // ===== FORWARD =====
  unsigned long start = millis();
  while (millis() - start < 4000)
  {
    driveForward(400);
  }

  stopMotors();
  delay(1000);

  // ===== TURN =====
  //turnLeft(500, 300);
  //delay(1000);

  // ===== STRAFE =====
  //strafeRight(400);
  //delay(2000);

  //stopMotors();
  //delay(3000);
}
