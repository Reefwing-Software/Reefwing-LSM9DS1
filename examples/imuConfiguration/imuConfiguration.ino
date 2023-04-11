/******************************************************************
  @file       imuConfiguration.ino
  @brief      Demonstrates how to change the LSM9DS1 settings
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     1.0.0
  Date:        20/03/23

  1.0.0     Original Release.       20/03/23

******************************************************************/

#include <ReefwingLSM9DS1.h>

ReefwingLSM9DS1 imu;

int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;
Configuration config;

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 

    //  Example non-default Configuration
    imu.setGyroScale(FS_500DPS);
    imu.setGyroBandwidth(MAXIMUM);
    imu.setGyroODR(GODR_476Hz);
    imu.setAccelScale(FS_XL_16G);
    imu.setMagScale(FS_16G);
    imu.setMagODR(MODR_80Hz);

    imu.start();
    imu.calibrateGyro();
    imu.calibrateAccel();
    imu.calibrateMag();
    delay(20)
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }
}

void loop() {
  imu.updateSensorData();

  if (millis() - previousMillis >= displayPeriod) {
    //  Display sensor data every displayPeriod, non-blocking.
    Serial.print("Gyro X: ");
    Serial.print(imu.data.gx);
    Serial.print("\tGyro Y: ");
    Serial.print(imu.data.gy);
    Serial.print("\tGyro Z: ");
    Serial.print(imu.data.gz);
    Serial.print(" DPS");
  
    Serial.print("\tLoop Frequency: ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");

    Serial.print("Accel X: ");
    Serial.print(imu.data.ax);
    Serial.print("\tAccel Y: ");
    Serial.print(imu.data.ay);
    Serial.print("\tAccel Z: ");
    Serial.print(imu.data.az);
    Serial.println(" G'S");

    Serial.print("Mag X: ");
    Serial.print(imu.data.mx);
    Serial.print("\tMag Y: ");
    Serial.print(imu.data.ay);
    Serial.print("\tMag Z: ");
    Serial.print(imu.data.az);
    Serial.println(" gauss\n");

    loopFrequency = 0;
    previousMillis = millis();
  }

  loopFrequency++;
}
