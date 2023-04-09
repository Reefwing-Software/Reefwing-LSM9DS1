/******************************************************************
  @file       simpleAccelerometer.ino
  @brief      Display the LSM9DS1 Accelerometer sensor readings
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
ScaledData accel;

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
    imu.start();
    imu.calibrateAccel();
    delay(20);
    //  Flush first reading
    imu.readAccel();
  } 
  else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }

  if (useSerialMonitor) {
    Serial.println("LSM9DS1 IMU Connected.");
    Serial.println("\nDefault Accelerometer Configuration used:");
    Serial.println("  - Full Scale: ± 8 g");
    Serial.println("  - Sample Rate (ODR): 119 Hz\n");
  }
  else {
    Serial.println("X \t Y \t Z");
  }
}

void loop() {
  //  Read Accelerometer
  if (imu.accelAvailable()) {
    accel = imu.readAccel();
  
    if (useSerialMonitor) {
      if (millis() - previousMillis >= displayPeriod) {
        //  Display sensor data every displayPeriod, non-blocking.
        Serial.print("Accel X: ");
        Serial.print(accel.sx);
        Serial.print("\tAccel Y: ");
        Serial.print(accel.sy);
        Serial.print("\tAccel Z: ");
        Serial.print(accel.sz);
        Serial.print(" G'S");
      
        Serial.print("\tLoop Frequency: ");
        Serial.print(loopFrequency);
        Serial.println(" Hz");
  
        loopFrequency = 0;
        previousMillis = millis();
      }
    }
    else {
      Serial.print(accel.sx);
      Serial.print('\t');
      Serial.print(accel.sy);
      Serial.print('\t');
      Serial.println(accel.sz);
    }
  }

  loopFrequency++;
}
