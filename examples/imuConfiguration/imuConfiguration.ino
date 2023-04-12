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

Configuration config;

void printConfig() {
  Serial.println("\n  ========  IMU Configuration    ========");
  Serial.print("Gyro/Accel FIFO Enabled: "); Serial.println(config.gyroAccelFIFOEnabled ? "True" : "False");
  Serial.print("FIFO Mode (0 = BYPASS, 1 = FIFO_THS, 3 = CONT_TO_FIFO, 4 = BYPASS_TO_CONT, 6 = FIFO_CONT): "); 
  Serial.println(config.fifoMode);
  Serial.print("FIFO Threshold: "); Serial.println(config.fifoThreshold);
  Serial.print("Gyro/Accel Operating Mode (0 = OFF, 1 = ACCEL, 2 = NORMAL, 3 = LOW_PWR): "); 
  Serial.println(config.gyroAccelOpMode);

  Serial.println("\n  ========  GYRO Configuration    ========");
  Serial.print("Power Down: "); Serial.print(config.gyro.powerDown ? "True" : "False");
  Serial.print(", Sleep Enabled: "); Serial.print(config.gyro.sleepEnabled ? "True" : "False");
  Serial.print(", Low Power Enabled: "); Serial.println(config.gyro.lowPowerEnabled ? "True" : "False");
  Serial.print("Full Scale (0 = 245, 1 = 500, 2 = 2000 DPS): "); Serial.println(config.gyro.scale);
  Serial.print("ODR (0 = OFF, 1 = 14.9, 2 = 59.5, 3 = 119, 4 = 238, 5 = 476, 6 = 952 Hz): "); 
  Serial.println(config.gyro.sampleRate);
  Serial.print("Bandwidth (0 = LOW, 1 = MID, 2 = HIGH, 3 = MAX): "); Serial.println(config.gyro.bandwidth);
  Serial.print("Orientation: (0 = XYZ): "); Serial.print(config.gyro.orientation);
  Serial.print(", Sign (0 = NONE): "); Serial.println(config.gyro.reverseSign);
  Serial.print("Raw Bias Offset, X: "); Serial.print(config.gyro.bias.x);
  Serial.print(", Y: "); Serial.print(config.gyro.bias.y);
  Serial.print(", Z: "); Serial.println(config.gyro.bias.z);

  Serial.println("\n  ========  ACCEL Configuration    ========");
  Serial.print("Power Down (True in NORMAL mode): "); Serial.println(config.accel.powerDown ? "True" : "False");
  Serial.print("Enable Auto Bandwidth: "); Serial.println(config.accel.autoBandwidthEnable ? "True" : "False");
  Serial.print("Scale (0 = 2, 1 = 16, 2 = 4, 3 = 8 G's): "); Serial.println(config.accel.scale);
  Serial.print("ODR (0 = OFF, 1 = 10, 2 = 50, 3 = 119, 4 = 238, 5 = 476, 6 = 952, 7 = Gyro ODR): "); 
  Serial.println(config.accel.sampleRate);
  Serial.print("Bandwidth (0 = 408, 1 = 211, 2 = 105, 3 = 50 Hz): "); 
  Serial.println(config.accel.bandwidth);
  Serial.print("Raw Bias Offset, X: "); Serial.print(config.accel.bias.x);
  Serial.print(", Y: "); Serial.print(config.accel.bias.y);
  Serial.print(", Z: "); Serial.println(config.accel.bias.z);

  Serial.println("\n  ========  MAG Configuration    ========");
  Serial.print("Fast ODR Enabled: "); Serial.print(config.mag.fastODR ? "True" : "False");
  Serial.print(", Temp Compensation Enabled: "); Serial.println(config.mag.temperatureCompensated ? "True" : "False");
  Serial.print("Operating Mode (0 = LOW, 1 = MED, 2 = HIGH, 3 = ULTRA): "); 
  Serial.println(config.mag.opMode);
  Serial.print("Sample Mode (0 = CONT, 1 = SINGLE, 2 = IDLE): "); 
  Serial.println(config.mag.sampleMode);
  Serial.print("Scale (0 = 4, 1 = 8, 2 = 12, 3 = 16 gauss): "); 
  Serial.println(config.mag.scale);
  Serial.println("ODR (0 = 0.625, 1 = 1.25, 2 = 2.5, 3 = 5, 4 = 10, 5 = 20, 6 = 40, 7 = 80 Hz..."); 
  Serial.print("...FAST ODR 8 = 155, 9 = 300, 10 = 560, 11 = 1000 Hz): ");
  Serial.println(config.mag.sampleRate);
  Serial.print("Raw Bias Offset, X: "); Serial.print(config.mag.bias.x);
  Serial.print(", Y: "); Serial.print(config.mag.bias.y);
  Serial.print(", Z: "); Serial.println(config.mag.bias.z);

  Serial.println("\n  ========  TEMPERATURE Configuration    ========");
  Serial.print("Temp Offset: "); Serial.print(config.temp.offset); Serial.println("°C\n");
  Serial.println("-----------------------------------------------------------------------------\n");
}

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 
    Serial.println("Default Configuration.\n"); 

    //  Read and display default configuration
    config = imu.getConfig();
    printConfig();

    //  Example non-default configuration
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

    Serial.println("NEW Configuration.\n");
    config = imu.getConfig();
    printConfig();

    delay(20);
    //  Flush the first reading - this is important!
    //  Particularly after changing the configuration.
    imu.readGyro();
    imu.readAccel();
    imu.readMag();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }
}

void loop() { }