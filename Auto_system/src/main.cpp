/* Encoder Library - Four Wheel Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * Expanded from the TwoKnobs example used by:
 * https://ros-mobile-robots.com/DG01D-E-motor-with-encoder/
 */

#include <Encoder.h>
#include "MotorConfig.h"

Encoder encoderFL(
    MOTOR_ENCODER_FRONT_LEFT_A_PIN,
    MOTOR_ENCODER_FRONT_LEFT_B_PIN
);
Encoder encoderFR(
    MOTOR_ENCODER_FRONT_RIGHT_A_PIN,
    MOTOR_ENCODER_FRONT_RIGHT_B_PIN
);
Encoder encoderRL(
    MOTOR_ENCODER_REAR_LEFT_A_PIN,
    MOTOR_ENCODER_REAR_LEFT_B_PIN
);
Encoder encoderRR(
    MOTOR_ENCODER_REAR_RIGHT_A_PIN,
    MOTOR_ENCODER_REAR_RIGHT_B_PIN
);

void setup() {
  Serial.begin(115200);
  Serial.println("DiffBot Four Wheel Encoders:");
}

long positionFL = -999;
long positionFR = -999;
long positionRL = -999;
long positionRR = -999;

void loop() {
  long newFL = encoderFL.read();
  long newFR = encoderFR.read();
  long newRL = encoderRL.read();
  long newRR = encoderRR.read();

  if (newFL != positionFL || newFR != positionFR || newRL != positionRL || newRR != positionRR) {
    Serial.print("FL = ");
    Serial.print(newFL);
    Serial.print(", FR = ");
    Serial.print(newFR);
    Serial.print(", RL = ");
    Serial.print(newRL);
    Serial.print(", RR = ");
    Serial.print(newRR);
    Serial.println();
    positionFL = newFL;
    positionFR = newFR;
    positionRL = newRL;
    positionRR = newRR;
  }

  // if a character is sent from the serial monitor,
  // reset all back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset all wheel encoders to zero");
    encoderFL.write(0);
    encoderFR.write(0);
    encoderRL.write(0);
    encoderRR.write(0);
  }
}
