/******************************************************************
  @file       readTemperature.ino
  @brief      Read the LSM9DS1 Gyro chip temperature
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

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 
    imu.start();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }
}

void loop() {
  //  Read chip temperature
  float temperature = 0.0;
  
  if (imu.tempAvailable) {
    temperature = imu.readTemperature();
  }

  //  Display sensor data every displayPeriod, non-blocking.
  if (millis() - previousMillis >= displayPeriod) {
      
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" C");
    
    Serial.print("\tLoop Frequency: ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");

    loopFrequency = 0;
    previousMillis = millis();
  }

  loopFrequency++;
}
