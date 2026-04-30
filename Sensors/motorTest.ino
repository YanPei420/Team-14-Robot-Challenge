#define Wire Wire1
#include <Motoron.h>

// Create two Motoron objects with different addresses
MotoronI2C mc1(16);  // First board
MotoronI2C mc2(17);  // Second board

void setup()
{
  Wire.begin();

  // Initialize both controllers
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();

  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();

  // Optional: acceleration settings
  mc1.setMaxAcceleration(1, 200);
  mc1.setMaxAcceleration(2, 200);
  mc2.setMaxAcceleration(1, 200);
  mc2.setMaxAcceleration(2, 200);
}

void loop()
{
  // Set all motors forward
  mc1.setSpeed(1, 400); // Motor 1
  mc1.setSpeed(2, 400); // Motor 2

  mc2.setSpeed(1, 400); // Motor 3
  mc2.setSpeed(2, 400); // Motor 4
}
