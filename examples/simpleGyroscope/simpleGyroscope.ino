/******************************************************************
  @file       simpleGyroscope.ino
  @brief      Display the LSM9DS1 Gyroscope rate sensor readings
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:    1.0.2
  Date:       10/06/23

  1.0.0   Original Release.           20/03/23
  1.0.1   Sensor Time Stamp added     13/04/23
  1.0.2   IMU Calibration bug fix     10/06/23

******************************************************************/

#include <ReefwingLSM9DS1.h>

ReefwingLSM9DS1 imu;

int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;
ScaledData gyro;

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
    Serial.println("Calibrating IMU.");
    imu.start();
    BiasOffsets bias = imu.calibrateGyro();
    Serial.print("Bias Offsets x: ");
    Serial.print(bias.x);
    Serial.print(", y: ");
    Serial.print(bias.y);
    Serial.print(", z: ");
    Serial.println(bias.z);
    delay(20);
    //  Flush first reading
    imu.readGyro();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }

  if (useSerialMonitor) {
    Serial.println("\nLSM9DS1 IMU Connected.");
    Serial.println("\nDefault Gyro Configuration used:");
    Serial.println("  - Full Scale: 2000 DPS");
    Serial.println("  - Sample Rate (ODR): 119 Hz\n");
  }
  else {
    Serial.println("X \t Y \t Z");
  }
}

void loop() {
  //  Read Gyroscope
  if (imu.gyroAvailable()) {
    gyro = imu.readGyro();
  
    if (useSerialMonitor) {
      if (millis() - previousMillis >= displayPeriod) {
        //  Display sensor data every displayPeriod, non-blocking.
        Serial.print("Gyro X: ");
        Serial.print(gyro.sx);
        Serial.print("\tGyro Y: ");
        Serial.print(gyro.sy);
        Serial.print("\tGyro Z: ");
        Serial.print(gyro.sz);
        Serial.print(" DPS");
      
        Serial.print("\tLoop Frequency: ");
        Serial.print(loopFrequency);
        Serial.println(" Hz");
  
        loopFrequency = 0;
        previousMillis = millis();
      }
    }
    else {
      Serial.print(gyro.sx);
      Serial.print('\t');
      Serial.print(gyro.sy);
      Serial.print('\t');
      Serial.println(gyro.sz);
    }
  }

  loopFrequency++;
}