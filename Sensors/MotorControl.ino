#define Wire Wire1
#include <Motoron.h>

MotoronI2C mc;

// Encoder pins
#define LEFT_ENC_A 2
#define LEFT_ENC_B 4
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 5

volatile long leftCount = 0;
volatile long rightCount = 0;

// Interrupt functions
void leftEncoderISR()
{
  if (digitalRead(LEFT_ENC_B))
    leftCount++;
  else
    leftCount--;
}

void rightEncoderISR()
{
  if (digitalRead(RIGHT_ENC_B))
    rightCount++;
  else
    rightCount--;
}

void stopMotors()
{
  mc.setSpeed(1,0);
  mc.setSpeed(2,0);
}

void driveForward(int baseSpeed)
{
  leftCount = 0;
  rightCount = 0;
  // Error between wheels
  long error = leftCount - rightCount;

  // Simple proportional correction
  float Kp = 2.0;
  int correction = Kp * error;

  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // Limit speeds
  leftSpeed = constrain(leftSpeed, -800, 800);
  rightSpeed = constrain(rightSpeed, -800, 800);

  mc.setSpeed(1, leftSpeed);
  mc.setSpeed(2, rightSpeed);
}

void turnLeft(long targetTicks, int speed)
{
  leftCount = 0;
  rightCount = 0;

  while (abs(leftCount)<targetTicks)
  {
    mc.setSpeed(1,-speed);
    mc.setSpeed(2,speed);
    delay(5);
  }

  stopMotors();
}

void turnRight(long targetTicks, int speed)
{
  leftCount = 0;
  rightCount = 0;

  while (abs(rightCount)<targetTicks){
    mc.setSpeed(1,speed);
    mc.setSpeed(2,-speed);
    delay(5);
  }

  stopMotors();
}

void setup()
{
  Wire.begin();

  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();

  // Encoder pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);
}

void loop()
{
  for (int i = 0; i < 200; i++)
  {
    driveForward(400);
    delay(20);
  }

  stopMotors();
  delay(1000);
  turnLeft(500,300);
  delay(1000);
}
