/******************************************************************
  @file       selfTest.ino
  @brief      Use the LSM9DS1 IMU Self Test Functionality
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     1.0.0
  Date:        20/03/23

  1.0.0     Original Release.       20/03/23

******************************************************************/

#include <ReefwingLSM9DS1.h>

ReefwingLSM9DS1 imu;
SelfTestResults results;

bool sensorInRange(float x, float y, float z, float min, float max) {
  return ((min <= x && x <= max) &&
          (min <= y && y <= max) &&
          (min <= z && z <= max)
         );
}

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 
    imu.start();
    delay(20);
    //  Flush first readings
    imu.updateSensorData();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }

  Serial.println("\nGyroscope SELF TEST");
  Serial.println("Place the Nano 33 BLE on a flat surface and don't move it.");

  results = imu.selfTestGyroAccel();

  Serial.print("x-axis: "); 
  Serial.print(results.gyrodx); 
  Serial.println(" dps"); 

  Serial.print("y-axis: "); 
  Serial.print(results.gyrody); 
  Serial.println(" dps"); 

  Serial.print("z-axis: "); 
  Serial.print(results.gyrodz); 
  Serial.println(" dps"); 

  Serial.println("Gyroscope x, y and z axis expected range: 20 - 250 dps");

  if (sensorInRange(results.gyrodx, results.gyrody, results.gyrodz, 20.0, 250.0)) {
    Serial.println("GYROSCOPE PASSED SELF-TEST");
  }
  else {
    Serial.println("GYROSCOPE FAILED SELF-TEST");
  }

  Serial.println("\nAccelerometer SELF TEST");
  Serial.print("x-axis: "); 
  Serial.print(results.accdx); 
  Serial.println(" mg"); 

  Serial.print("y-axis = "); 
  Serial.print(results.accdy); 
  Serial.println(" mg"); 

  Serial.print("z-axis = "); 
  Serial.print(results.accdz); 
  Serial.println(" mg"); 

  Serial.println("Accelerometer x, y, and z axis expected range: 60 - 1700 mg");

  if (sensorInRange(results.accdx, results.accdy, results.accdz, 60.0, 1700.0)) {
    Serial.println("ACCELEROMETER PASSED SELF-TEST");
  }
  else {
    Serial.println("ACCELEROMETER FAILED SELF-TEST");
  }
}

void loop() { }