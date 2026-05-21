#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

LSM6 imu;
LIS3MDL mag;

void setup(){
  Serial.begin(115200);
  Wire1.begin();

  imu.setBus(&Wire1);
  if (!imu.init())
  {
    Serial.println("Failed to detect LSM6DS33");
    while (1);
  }
  imu.enableDefault();

  mag.setBus(&Wire1);
  if (!mag.init())
  {
    Serial.println("Failed to detect LIS3MDL");
    while (1);
  }
  mag.enableDefault();

  Serial.println("IMU initialised");
}

void loop(){
  imu.read();
  mag.read();

  // // acceleration magnitude
  // double mag_acc = sqrt(pow(imu.a.x, 2) + pow(imu.a.y, 2) + pow(imu.a.z, 2));

  // // tilt magnituse
  // double mag_tilt =  sqrt(pow(imu.g.y, 2) + pow(imu.g.z, 2));
  
  
  // Serial.print("Acceleration Magnitude: ");
  // Serial.println(mag_acc);
  // Serial.print("Tilt Magnitude: ");
  // Serial.println(mag_tilt);

  // Accelerometer
  Serial.print("Accel (mg): ");
  Serial.print(imu.a.x); Serial.print(", ");
  Serial.print(imu.a.y); Serial.print(", ");
  Serial.println(imu.a.z);

  // Gyroscope
  Serial.print("Gyro (dps): ");
  Serial.print(imu.g.x); Serial.print(", ");
  Serial.print(imu.g.y); Serial.println(", ");
  Serial.println(imu.g.z);

  // Magnetometer
  Serial.print("Mag (mG): ");
  Serial.print(mag.m.x); Serial.print(", ");
  Serial.print(mag.m.y); Serial.print(", ");
  Serial.println(mag.m.z);

  Serial.println();
  delay(100);
}
