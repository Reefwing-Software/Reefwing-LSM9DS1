/******************************************************************
  @file       simpleMagnetometer.ino
  @brief      Display the LSM9DS1 Magnetometer sensor readings
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
ScaledData mag;

//  Default to displaying gyro values to the Serial Monitor
//  Set to false if you want to use the Arduino Serial Plotter
bool useSerialMonitor = true;

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
    //  Flush first reading
    imu.readMag();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }

  if (useSerialMonitor) {
    Serial.println("\nDefault Magnetometer Configuration used:");
    Serial.println("  - Full Scale: ± 4 Gauss");
    Serial.println("  - Sample Rate (ODR): 10 Hz\n");
  }
  else {
    Serial.println("X \t Y \t Z");
  }
}

void loop() {
  //  Read Magnetometer
  if (imu.magAvailable()) {
    mag = imu.readMag();
  
    if (useSerialMonitor) {
      if (millis() - previousMillis >= displayPeriod) {
        //  Display sensor data every displayPeriod, non-blocking.
        Serial.print("Mag X: ");
        Serial.print(mag.sx);
        Serial.print("\tMag Y: ");
        Serial.print(mag.sy);
        Serial.print("\tMag Z: ");
        Serial.print(mag.sz);
        Serial.print(" gauss");
      
        Serial.print("\tLoop Frequency: ");
        Serial.print(loopFrequency);
        Serial.println(" Hz");
  
        loopFrequency = 0;
        previousMillis = millis();
      }
    }
    else {
      Serial.print(mag.sx);
      Serial.print('\t');
      Serial.print(mag.sy);
      Serial.print('\t');
      Serial.println(mag.sz);
    }
  }

  loopFrequency++;
}

