/******************************************************************
  @file       ReefwingLSM9DS1.h
  @brief      Arduino Library for the LSM9DS1 9-axis IMU
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:       David Such
  Version:    1.0.1
  Date:       13/04/23

  1.0.0   Original Release.           20/03/23
  1.0.1   Sensor Time Stamp added     13/04/23

******************************************************************/

#ifndef ReefwingLSM9DS1_h
#define ReefwingLSM9DS1_h

#include <Arduino.h>
#include <Wire.h>

#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

/******************************************************************
 *
 * LSM9DS1 Class Definition - 
 * 
 ******************************************************************/

class ReefwingLSM9DS1 {
public:
  ReefwingLSM9DS1();

  void begin(TwoWire *wire = &WIRE);
  void start();
  void reset();
  bool connected();
  void updateSensorData();
  void updateTempData();

  void enableLowPower(bool bitValue);
  void enableGyroSleep(bool bitValue);
  void enableAccelAutoBandwidth(bool bitValue);
  void enableMagFastODR(bool bitValue);
  void enableMagTempComp(bool bitValue);
  void enableFIFO(bool bitValue);

  void setTempOffset(float offset = 25.0f);
  void setBiasOffset(SensorType sensor, BiasOffsets bias);
  void setFIFOMode(FIFOMode mode, uint8_t threshold);
  uint8_t getFIFOSampleNumber();

  void reverseGyroSign(GyroReverseSign axis);
  void setGyroOrientation(GyroOrient orient);

  void setSampleMode(MagSampleModes mode);
  void setMagOperatingMode(MagOpModes mode);

  void setGyroScale(GyroScale scale);
  void setAccelScale(AccelScale scale);
  void setMagScale(MagScale scale);

  void setGyroODR(GyroODR rate);
  void setAccelODR(AccelODR rate);
  void setMagODR(MagODR rate);

  void setGyroBandwidth(GyroBW bandwidth);
  void setAccelBandwidth(AccelBW bandwidth);

  bool gyroAvailable();
  bool accelAvailable();
  bool magAvailable();
  bool tempAvailable();

  float readTemp(TempScale scale = TempScale::CELSIUS);
  ScaledData readGyro();
  RawData readGyroRaw();
  ScaledData readAccel();
  RawData readAccelRaw();
  ScaledData readMag();
  RawData readMagRaw();

  BiasOffsets calibrateGyro(bool save = true);
  BiasOffsets calibrateGyroFIFO(bool save = true);
  BiasOffsets calibrateAccel(bool save = true);
  BiasOffsets calibrateAccelFIFO(bool save = true);
  BiasOffsets calibrateMag(bool save = true);
  BiasOffsets getBiasOffset(SensorType sensor);

  SelfTestResults selfTestGyroAccel();
  MagTestResults selfTestMag();

  Configuration getConfig();
  int16_t twosCompToInteger(uint16_t two_compliment_val);

  SensorData data;
  TempData gyroTemp;

private:
  uint8_t whoAmIGyro();
  uint8_t whoAmIMag();
  uint8_t readByte(uint8_t address, uint8_t regAddress);
  void readBytes(uint8_t address, uint8_t regAddress, uint8_t numBytes, uint8_t * dest);
  void writeByte(uint8_t address, uint8_t regAddress, uint8_t data);

  BiasOffsets averageMagOffsets();

  TwoWire* _wire;               //  I2C Bus
  Configuration _config;        //  IMU Configuration Values
  float _aRes, _gRes, _mRes;    //  Resolution for each sensor

};

#endif