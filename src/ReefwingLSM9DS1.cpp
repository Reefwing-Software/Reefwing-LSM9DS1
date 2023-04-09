/******************************************************************
  @file       ReefwingLSM9DS1.cpp
  @brief      Arduino Library for the LSM9DS1 9-axis IMU
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     1.0.0
  Date:        20/03/23

  1.0.0     Original Release.       20/03/23

******************************************************************/

#include "ReefwingLSM9DS1.h"

/******************************************************************
 *
 * LSM9DS1 Implementation - 
 * 
 ******************************************************************/

ReefwingLSM9DS1::ReefwingLSM9DS1() { }

void ReefwingLSM9DS1::reset() {
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG8, 0x05);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, 0x0c);
  delay(20);

  //  Check that chip boot up is complete - bit 3 is BOOT_STATUS
  //  Mask the STATUS_REG with 0b00001000 = 0x08
  while ((readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_STATUS_REG) & 0x08) != 0) {
    yield();
  }
}

void ReefwingLSM9DS1::begin(TwoWire *wire) {
  _wire = wire;
  _wire->begin();
  reset();

  //  Default configuration
  enableLowPower(false);
  enableAccelAutoBandwidth(true);
  setSampleMode(CONTINUOUS);
  setGyroScale(FS_2000DPS);
  setAccelScale(FS_XL_8G);
  setMagScale(FS_4G);
  setGyroBandwidth(LOWEST);
  setGyroODR(GODR_119Hz);    //  NORMAL Op Mode, gyro rate = accel rate
  setMagODR(MODR_10Hz);

  //  Flush the first 20 readings to allow for sensor turn on
  //  See Tables 11 and 12 in the LSM9DS1 Data Sheet
  for (int ctr = 0; ctr < FLUSH_SAMPLES; ctr++) {
    readGyroRaw();
    readAccelRaw();
    readMagRaw();
    delay(5);
  }
}

void ReefwingLSM9DS1::start() {
  //  Initialise the Gyro - assumes configuration is done
  //  Control Register 4 - enable gyro axis output bits (5, 4 & 3)
  //  The other bits default to 0. CTRL_REG4 = 0b00111000 = 0x38
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG4, 0x38);

  //  Initialise the Accelerometer
  //  Control Register 5 XL - enable accel axis output bits (5, 4 & 3)
  //  The other bits default to 0. CTRL_REG5_XL = 0b00111000 = 0x38
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG5_XL, 0x38);
  
  //  Enable block data update, allow auto-increment during multiple byte read, 
  //  data LSB @ lower address. CTRL_REG8 = 0b01000100 = 0x44
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG8, 0x44);

  //  Initialise the Magnetometer
  //  Enable block data update
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 );
}

uint8_t ReefwingLSM9DS1::whoAmIGyro() {
    // Read WHO_AM_I register for LSM9DS1 accel/gyro
    return readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_WHO_AM_I);  
}

uint8_t ReefwingLSM9DS1::whoAmIMag() {
    // Read WHO_AM_I register for LSM9DS1 magnetometer
    return readByte(LSM9DS1M_ADDRESS, LSM9DS1M_WHO_AM_I);  
}

bool ReefwingLSM9DS1::connected() {
  return (whoAmIGyro() == LSM9DS1AG_WHO_AM_I_VALUE &&
          whoAmIMag() == LSM9DS1M_WHO_AM_I_VALUE);
}

void ReefwingLSM9DS1::updateSensorData() {
  //  Updates the class SensorData structure if data is available

  if (gyroAvailable()) {  
    ScaledData gyr = readGyro(); 

    data.gx = gyr.sx;
    data.gy = gyr.sy;
    data.gz = gyr.sz; 
  }

  if (accelAvailable()) {  
    ScaledData acc = readAccel(); 

    data.ax = acc.sx;
    data.ay = acc.sy;
    data.az = acc.sz;
  }

  if (magAvailable()) {  
    ScaledData mag = readMag();   

    data.mx = mag.sx;
    data.my = mag.sy;
    data.mz = mag.sz;
  }
}

/******************************************************************
 *
 * LSM9DS1 Configuration - 
 * 
 ******************************************************************/

void ReefwingLSM9DS1::enableFIFO(bool bitValue) {
  uint8_t CTRL_REG9 = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG9);

  //  Clear the FIFO_EN bit (1) in CTRL_REG9, maintain the rest
  //  0xFD = 0b11111101, FIFO_EN = 0, FIFO disabled (default)
  CTRL_REG9 &= 0xFD;

  if (bitValue) {
    CTRL_REG9 |= (0x01 << 1);
  }

  _config.gyroAccelFIFOEnabled = bitValue;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG9, CTRL_REG9);
}

void ReefwingLSM9DS1::enableGyroSleep(bool bitValue) {
  uint8_t CTRL_REG9 = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG9);

  //  Clear the SLEEP_G bit (6) in CTRL_REG9, maintain the rest
  //  0xBF = 0b10111111, SLEEP_G = 0, gyro sleep mode disabled (default)
  CTRL_REG9 &= 0xBF;

  if (bitValue) {
    CTRL_REG9 |= (0x01 << 6);
  }

  _config.gyro.sleepEnabled = bitValue;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG9, CTRL_REG9);
}

void ReefwingLSM9DS1::enableAccelAutoBandwidth(bool bitValue) {
  uint8_t CTRL_REG6_XL = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG6_XL);

  //  Clear the BW_SCAL_ODR bit (2), maintain the rest
  //  0xFB = 0b11111011, BW_SCAL_ODR = 0, auto bandwith enabled (default)
  CTRL_REG6_XL &= 0xFB;

  if (bitValue) {
    CTRL_REG6_XL |= (0x01 << 2);
  }

  _config.accel.autoBandwidthEnable = bitValue;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG6_XL, CTRL_REG6_XL);
}

void ReefwingLSM9DS1::enableLowPower(bool bitValue) {
  uint8_t CTRL_REG3_G = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG3_G);

  //  Clear the LP_mode bit (7), maintain the rest
  //  0x7F = 0b01111111, LP_mode = 0, low-power disabled
  CTRL_REG3_G &= 0x7F;

  if (bitValue) { //  Enable LOW POWER mode
    CTRL_REG3_G |= (0x01 << 7);
    if (_config.gyroAccelOpMode == NORMAL && _config.gyro.sampleRate < GODR_238Hz) {
      _config.gyroAccelOpMode = LOW_POWER;
    }
  }
  else {
    if (_config.gyroAccelOpMode == LOW_POWER) {
      _config.gyroAccelOpMode = NORMAL;
    }
  }

  _config.gyro.lowPowerEnabled = bitValue;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG3_G, CTRL_REG3_G);
}

void ReefwingLSM9DS1::enableMagFastODR(bool bitValue) {
  uint8_t CTRL_REG1_M = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M);

  //  Clear the FAST_ODR bit (1), maintain the rest
  //  0xFD = 0b11111101, FAST_ODR = 0, fast ODR disabled (default)
  CTRL_REG1_M &= 0xFD;

  if (bitValue) {
    CTRL_REG1_M |= (0x01 << 1);
  }

  _config.mag.fastODR = bitValue;
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, CTRL_REG1_M);
}

void ReefwingLSM9DS1::enableMagTempComp(bool bitValue) {
  uint8_t CTRL_REG1_M = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M);

  //  Clear the TEMP_COMP bit (7) in CTRL_REG1_M, maintain the rest
  //  0x7F = 0b01111111, TEMP_COMP = 0, temperature compensation disabled (default)
  CTRL_REG1_M &= 0x7F;

  if (bitValue) {
    CTRL_REG1_M |= (0x01 << 7);
  }

  _config.mag.temperatureCompensated = bitValue;
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, CTRL_REG1_M);
}

void ReefwingLSM9DS1::reverseGyroSign(GyroReverseSign axis) {
  uint8_t ORIENT_CFG_G = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_ORIENT_CFG_G);

  //  Clear the SignX_G, SignY_G, and SignZ_G bits (5, 4, & 3), maintain the rest
  //  0xC7 = 0b11000111, SignXYZ_G = 000, sign reverse disabled (default)
  ORIENT_CFG_G &= 0xC7;

  switch(axis) {
    case GyroReverseSign::SIGN_Z:         //  SignXYZ_G = 001
      ORIENT_CFG_G |= (0x01 << 3);
      break;
    case GyroReverseSign::SIGN_Y:         //  SignXYZ_G = 010
      ORIENT_CFG_G |= (0x02 << 3);
      break;
    case GyroReverseSign::SIGN_Y_Z:       //  SignXYZ_G = 011
      ORIENT_CFG_G |= (0x03 << 3);
      break;
    case GyroReverseSign::SIGN_X:         //  SignXYZ_G = 100
      ORIENT_CFG_G |= (0x04 << 3);
      break;
    case GyroReverseSign::SIGN_X_Z:       //  SignXYZ_G = 101
      ORIENT_CFG_G |= (0x05 << 3);
      break;
    case GyroReverseSign::SIGN_X_Y:       //  SignXYZ_G = 110
      ORIENT_CFG_G |= (0x06 << 3);
      break;
    case GyroReverseSign::SIGN_X_Y_Z:     //  SignXYZ_G = 111
      ORIENT_CFG_G |= (0x07 << 3);
      break;
    default:                              //  SignXYZ_G = 000
      break;
  }

  _config.gyro.reverseSign = axis;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_ORIENT_CFG_G, ORIENT_CFG_G); 
}

void ReefwingLSM9DS1::setFIFOMode(FIFOMode mode, uint8_t threshold) {
  //  Maximum FIFO sample threshold, FTH = 31 (0x1F)
  uint8_t FTH = threshold <= 0x1F ? threshold : 0x1F;
  uint8_t FIFO_CTRL = (mode << 5) | FTH;

  _config.fifoMode = mode;
  _config.fifoThreshold = FTH;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_FIFO_CTRL, FIFO_CTRL);
}

uint8_t ReefwingLSM9DS1::getFIFOSampleNumber() {
  //  Mask out the first two bits (FTH and OVRN), to get FSS [5:0]
  //  0x3F = 0b00111111
  uint8_t FIFO_SRC = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_FIFO_SRC);

  return (FIFO_SRC & 0x3F);
}

void ReefwingLSM9DS1::setGyroOrientation(GyroOrient orient) {
  uint8_t ORIENT_CFG_G = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_ORIENT_CFG_G);

  //  Clear the ORIENT bits (2, 1, & 0), maintain the rest
  //  0xF8 = 0b11111000, ORIENT = 000, Pitch = X, Roll = Y, Yaw = Z (default)
  ORIENT_CFG_G &= 0xF8;

  switch(orient) {
    case GyroOrient::PRY_XZY:           //  ORIENT = 001
      ORIENT_CFG_G |= 0x01;
      break;
    case GyroOrient::PRY_YXZ:           //  ORIENT = 010
      ORIENT_CFG_G |= 0x02;
      break;
    case GyroOrient::PRY_YZX:           //  ORIENT = 011
      ORIENT_CFG_G |= 0x03;
      break;
    case GyroOrient::PRY_ZXY:           //  ORIENT = 100
      ORIENT_CFG_G |= 0x04;
      break;
    case GyroOrient::PRY_ZYX:           //  ORIENT = 101
      ORIENT_CFG_G |= 0x05;
      break;
  }

  _config.gyro.orientation = orient;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_ORIENT_CFG_G, ORIENT_CFG_G);
}

void ReefwingLSM9DS1::setSampleMode(MagSampleModes mode) {
  uint8_t CTRL_REG3_M = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M);

  //  Clear the MD bits (1 & 0) in CTRL_REG3_M, maintain the rest
  //  0xFC = 0b11111100, MD = 00, sample mode = continuous
  CTRL_REG3_M &= 0xFC;

  switch(mode) {
    case MagSampleModes::IDLE_OFF:          //  MD = 10 or MD = 11
      CTRL_REG3_M |= 0x02;
      break;
    case MagSampleModes::SINGLE_SHOT:       //  MD = 01
      CTRL_REG3_M |= 0x01;
      break;
    default:                                //  MD = 00, Continuous
      break;
  }

  _config.mag.sampleMode = mode;
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, CTRL_REG3_M);
}
  
void ReefwingLSM9DS1::setMagOperatingMode(MagOpModes mode) {
  uint8_t CTRL_REG1_M = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M);   //  x and y axis
  uint8_t CTRL_REG4_M = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M);   //  z axis

  //  Clear the OM bits (6 & 5) in CTRL_REG1_M, maintain the rest
  //  0x9F = 0b10011111, OM = 00, mode = low-perfomance (default)
  CTRL_REG1_M &= 0x9F;

  //  Clear the OMZ bits (3 & 2) in CTRL_REG4_M, maintain the rest
  //  0xF3 = 0b11110011, OMZ = 00, mode = low-perfomance (default)
  CTRL_REG4_M &= 0xF3;

  switch(mode) {
    case MagOpModes::MEDIUM:            //  OM = 01 & OMZ = 01
      CTRL_REG1_M |= (0x01 << 5);
      CTRL_REG4_M |= (0x01 << 2);
      break;
    case MagOpModes::HIGH_PERFORMANCE:  //  OM = 10 & OMZ = 10
      CTRL_REG1_M |= (0x02 << 5);
      CTRL_REG4_M |= (0x02 << 2);
      break;
    case MagOpModes::ULTRA:             //  OM = 11 & OMZ = 11
      CTRL_REG1_M |= (0x03 << 5);
      CTRL_REG4_M |= (0x03 << 2);
      break;
    default:
      break;
  }

  _config.mag.opMode = mode;
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, CTRL_REG1_M);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, CTRL_REG4_M);
}

void ReefwingLSM9DS1::setGyroScale(GyroScale scale) {
  uint8_t CTRL_REG1_G = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G);

  //  Clear the FS bits (3 & 4), maintain the rest
  //  0xE7 = 0b11100111, FS = 00, scale = 245 DPS
  CTRL_REG1_G &= 0xE7;

  switch(scale) {
    case GyroScale::FS_500DPS:
      CTRL_REG1_G |= (0x01 << 3);
      _gRes = SENSITIVITY_GYROSCOPE_500;
      break;
    case GyroScale::FS_2000DPS:
      CTRL_REG1_G |= (0x03 << 3);
      _gRes = SENSITIVITY_GYROSCOPE_2000;
      break;
    default:  //  Default scale is 245 DPS
      _gRes = SENSITIVITY_GYROSCOPE_245;
      break;
  }

  _config.gyro.scale = scale;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G, CTRL_REG1_G);
}

void ReefwingLSM9DS1::setAccelScale(AccelScale scale) {
  uint8_t CTRL_REG6_XL = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG6_XL);

  //  Clear the FS_XL bits (3 & 4), maintain the rest
  //  0xE7 = 0b11100111, FS_XL = 00, scale = ±2 g (default)
  CTRL_REG6_XL &= 0xE7;

  switch(scale) {
    case AccelScale::FS_XL_4G:
      CTRL_REG6_XL |= (0x02 << 3);
      _aRes = SENSITIVITY_ACCELEROMETER_4;
      break;
    case AccelScale::FS_XL_8G:
      CTRL_REG6_XL |= (0x03 << 3);
      _aRes = SENSITIVITY_ACCELEROMETER_8;
      break;
    case AccelScale::FS_XL_16G:
      CTRL_REG6_XL |= (0x01 << 3);
      _aRes = SENSITIVITY_ACCELEROMETER_16;
      break;
    default:  //  Default scale is ±2 g
    _aRes = SENSITIVITY_ACCELEROMETER_2;
  }

  _config.accel.scale = scale;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG6_XL, CTRL_REG6_XL);
}

void ReefwingLSM9DS1::setMagScale(MagScale scale) {
  uint8_t CTRL_REG2_M = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M);

  //  Clear the FS bits (5 & 6), maintain the rest
  //  0x9F = 0b10011111, FS = 00, scale = ± 4 gauss (default)
  CTRL_REG2_M &= 0x9F;

  switch(scale) {
    case MagScale::FS_8G:
      CTRL_REG2_M |= (0x01 << 5);
      _mRes = SENSITIVITY_MAGNETOMETER_8;
      break;
    case MagScale::FS_12G:
      CTRL_REG2_M |= (0x02 << 5);
      _mRes = SENSITIVITY_MAGNETOMETER_12;
      break;
    case MagScale::FS_16G:
      CTRL_REG2_M |= (0x03 << 5);
      _mRes = SENSITIVITY_MAGNETOMETER_16;
      break;
    default:  //  Default scale is ± 4 gauss
      _mRes = SENSITIVITY_MAGNETOMETER_4;
      break;
  }

  _config.mag.scale = scale;
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, CTRL_REG2_M);
  delay(20);
}

void ReefwingLSM9DS1::setGyroODR(GyroODR rate) {
  uint8_t CTRL_REG1_G = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G);

  //  Clear the ODR_G bits (7, 6 & 5), maintain the rest
  //  0x1F = 0b00011111, ODR_G = 000, mode = power-down
  CTRL_REG1_G &= 0x1F;

  _config.gyro.powerDown = false;
  _config.gyroAccelOpMode = NORMAL;

  switch(rate) {
    case GyroODR::GODR_14_9Hz:   //  ODR_G = 001
      CTRL_REG1_G |= (0x01 << 5);
      if (_config.gyro.lowPowerEnabled) {
        _config.gyroAccelOpMode = LOW_POWER;
      }
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_408Hz;
      }
      break;
    case GyroODR::GODR_59_5Hz:   //  ODR_G = 010
      CTRL_REG1_G |= (0x02 << 5);
      if (_config.gyro.lowPowerEnabled) {
        _config.gyroAccelOpMode = LOW_POWER;
      }
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_408Hz;
      }
      break;
    case GyroODR::GODR_119Hz:   //  ODR_G = 011
      CTRL_REG1_G |= (0x03 << 5);
      if (_config.gyro.lowPowerEnabled) {
        _config.gyroAccelOpMode = LOW_POWER;
      }
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_50Hz;
      }
      break;
    case GyroODR::GODR_238Hz:   //  ODR_G = 100
      CTRL_REG1_G |= (0x04 << 5);
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_105Hz;
      }
      break;
    case GyroODR::GODR_476Hz:   //  ODR_G = 101
      CTRL_REG1_G |= (0x05 << 5);
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_211Hz;
      }
      break;
    case GyroODR::GODR_952Hz:   //  ODR_G = 110
      CTRL_REG1_G |= (0x06 << 5);
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_408Hz;
      }
      break;
    default:  //  ODR_G = 000, default rate is power-down
      _config.gyro.powerDown = true;
      _config.gyroAccelOpMode = ACCELEROMETER_ONLY;
      if (_config.accel.powerDown) {
        _config.gyroAccelOpMode = POWERED_DOWN;
      }
      break;
  }

  _config.gyro.sampleRate = rate;
  if (_config.gyroAccelOpMode != ACCELEROMETER_ONLY) {
    _config.accel.sampleRate = AODR_GYRO_VALUE;
  }
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G, CTRL_REG1_G);
}

void ReefwingLSM9DS1::setAccelODR(AccelODR rate) {
  uint8_t CTRL_REG6_XL = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG6_XL);

  //  Clear the ODR_XL bits (7, 6 & 5), maintain the rest
  //  0x1F = 0b00011111, ODR_XL = 000, mode = power-down
  CTRL_REG6_XL &= 0x1F;

  _config.accel.powerDown = false;
  _config.gyroAccelOpMode = ACCELEROMETER_ONLY;

  switch(rate) {
    case AccelODR::AODR_10Hz:  //  ODR_XL = 001
      CTRL_REG6_XL |= (0x01 << 5);
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_408Hz;
      }
      break;
    case AccelODR::AODR_50Hz:  //  ODR_XL = 010
      CTRL_REG6_XL |= (0x02 << 5);
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_408Hz;
      }
      break;
    case AccelODR::AODR_119Hz: //  ODR_XL = 011
      CTRL_REG6_XL |= (0x03 << 5);
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_50Hz;
      }
      break;
    case AccelODR::AODR_238Hz: //  ODR_XL = 100
      CTRL_REG6_XL |= (0x04 << 5);
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_105Hz;
      }
      break;
    case AccelODR::AODR_476Hz: //  ODR_XL = 101
      CTRL_REG6_XL |= (0x05 << 5);
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_211Hz;
      }
      break;
    case AccelODR::AODR_952Hz: //  ODR_XL = 110
      CTRL_REG6_XL |= (0x06 << 5);
      if (_config.accel.autoBandwidthEnable) {
        _config.accel.bandwidth = BW_408Hz;
      }
      break;
    case AccelODR::AODR_GYRO_VALUE:
      //  If Op Mode = NORMAL, accel ODR = gyro ODR
    default:  //  ODR_XL = 000, default rate is power-down
      _config.accel.powerDown = true;
      _config.gyroAccelOpMode = NORMAL;
      if (_config.gyro.powerDown) {
        _config.gyroAccelOpMode = POWERED_DOWN;
      }
      else if (_config.gyro.lowPowerEnabled && _config.gyro.sampleRate < GODR_238Hz) {
        _config.gyroAccelOpMode = LOW_POWER;
      }
      break;
  }

  _config.accel.sampleRate = rate;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG6_XL, CTRL_REG6_XL);
}

void ReefwingLSM9DS1::setMagODR(MagODR rate) {

  if (rate > MODR_80Hz) {
    enableMagFastODR(true);

    switch(rate) {
      case MagODR::MODR_155Hz:       //  OM = 11, mode = ULTRA
        setMagOperatingMode(ULTRA);
        break;
      case MagODR::MODR_300Hz:       //  OM = 10, mode = HIGH_PERFORMANCE
        setMagOperatingMode(HIGH_PERFORMANCE);
        break;
      case MagODR::MODR_560Hz:       //  OM = 01, mode = MEDIUM
        setMagOperatingMode(MEDIUM);
        break;
      case MagODR::MODR_1000Hz:      //  OM = 00 (OM default), mode = LOW_PERFORMANCE
        setMagOperatingMode(LOW_PERFORMANCE);
        break;
    }
  } 
  else {
    enableMagFastODR(false);

    uint8_t CTRL_REG1_M = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M);

    //  Clear the DO bits (4, 3 & 2), maintain the rest
    //  0xE3 = 0b11100011, DO = 000, rate = 0.625 Hz
    CTRL_REG1_M &= 0xE3;

    switch(rate) {
      case MagODR::MODR_1_25Hz:      //  DO = 001
        CTRL_REG1_M |= (0x01 << 2);
        break;
      case MagODR::MODR_2_5Hz:       //  DO = 010
        CTRL_REG1_M |= (0x02 << 2);
        break;
      case MagODR::MODR_5Hz:         //  DO = 011
        CTRL_REG1_M |= (0x03 << 2);
        break;
      case MagODR::MODR_10Hz:        //  DO = 100
        CTRL_REG1_M |= (0x04 << 2);
        break;
      case MagODR::MODR_20Hz:        //  DO = 101
        CTRL_REG1_M |= (0x05 << 2);
        break;
      case MagODR::MODR_40Hz:        //  DO = 110
        CTRL_REG1_M |= (0x06 << 2);
        break;
      case MagODR::MODR_80Hz:        //  DO = 111
        CTRL_REG1_M |= (0x07 << 2);
        break;
      default:
        break;
    }

    _config.mag.sampleRate = rate;
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, CTRL_REG1_M);
  }
}

void ReefwingLSM9DS1::setGyroBandwidth(GyroBW bandwidth) {
  uint8_t CTRL_REG1_G = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G);

  //  Clear the BW_G bits (0 & 1), maintain the rest
  //  0xFC = 0b11111100, BW_G = 00, bandwidth = LOW (default)
  CTRL_REG1_G &= 0xFC;

  switch(bandwidth) {
    case GyroBW::MIDDLE:    //  BW_G = 01
      CTRL_REG1_G |= 0x01;
      break;
    case GyroBW::HIGHEST:   //  BW_G = 10
      CTRL_REG1_G |= 0x02;
      break;
    case GyroBW::MAXIMUM:   //  BW_G = 11
      CTRL_REG1_G |= 0x03;
      break;
    default:                //  BW_G = 00, bandwidth = LOW (default)
      break;
  }

  _config.gyro.bandwidth = bandwidth;
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G, CTRL_REG1_G);
}

void ReefwingLSM9DS1::setAccelBandwidth(AccelBW bandwidth) {
  if (!_config.accel.autoBandwidthEnable) {
    uint8_t CTRL_REG6_XL = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG6_XL);

    //  Clear the BW_XL bits (1 & 0), maintain the rest
    //  0xFC = 0b11111100, BW_XL = 00, bandwidth = 408 Hz (default)
    CTRL_REG6_XL &= 0xFC;

    switch(bandwidth) {
      case AccelBW::BW_50Hz:    //  BW_XL = 11
        CTRL_REG6_XL |= 0x03;
        break;
      case AccelBW::BW_105Hz:   //  BW_XL = 10
        CTRL_REG6_XL |= 0x02;
        break;
      case AccelBW::BW_211Hz:   //  BW_XL = 01
        CTRL_REG6_XL |= 0x01;
        break;
      default:                  //  BW_XL = 00, bandwidth = 408 Hz (default)
        break;
    }

    _config.accel.bandwidth = bandwidth;
    writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG6_XL, CTRL_REG6_XL);
  }
}

Configuration ReefwingLSM9DS1::getConfig() {
  return _config;
}

/******************************************************************
 *
 * LSM9DS1 Sensor Calibration - 
 * 
 ******************************************************************/

void ReefwingLSM9DS1::setTempOffset(float offset) {
  _config.temp.offset = offset;
}

BiasOffsets ReefwingLSM9DS1::calibrateGyro(bool save) {
  BiasOffsets bias;

  //  Average 32 zero-rate (bias offset) samples
  for (int i = 0; i < 32; i++) {
    while (!gyroAvailable()) {
      yield();
    }

    RawData gyr = readGyroRaw();

    bias.x += gyr.rx;
    bias.y += gyr.ry;
    bias.z += gyr.rz;
  }

  bias.x = bias.x / 32;
  bias.y = bias.y / 32;
  bias.z = bias.z / 32;

  if (save) {  setBiasOffset(GYROSCOPE, bias);  }

  return bias;
}

BiasOffsets ReefwingLSM9DS1::calibrateGyroFIFO(bool save) {
  BiasOffsets bias;

  //  Turn on FIFO and set sample threshold to 32
  enableFIFO(true);
  setFIFOMode(FIFO_THS, 0x1F);

  //  Wait for FIFO to fill
  while (getFIFOSampleNumber() < 0x1F) {
    yield();
  }

  //  Average 32 zero-rate (bias offset) samples
  for (int i = 0; i < 32; i++) {
    RawData gyr = readGyroRaw();

    bias.x += gyr.rx;
    bias.y += gyr.ry;
    bias.z += gyr.rz;
  }

  bias.x = bias.x / 32;
  bias.y = bias.y / 32;
  bias.z = bias.z / 32;

  if (save) {  setBiasOffset(GYROSCOPE, bias);  }
  enableFIFO(false);
  setFIFOMode(BYPASS, 0x00);

  return bias;
}

BiasOffsets ReefwingLSM9DS1::calibrateAccel(bool save) {
  BiasOffsets bias;

  //  Average 32 zero-rate (bias offset) samples
  for (int i = 0; i < 32; i++) {
    while (!accelAvailable()) {
      yield();
    }

    RawData acc = readAccelRaw();

    bias.x += acc.rx;
    bias.y += acc.ry;
    bias.z += acc.rz - (int16_t)(1.0f/_aRes);
  }

  bias.x = bias.x / 32;
  bias.y = bias.y / 32;
  bias.z = bias.z / 32;

  if (save) {  setBiasOffset(ACCELEROMETER, bias);  }

  return bias;
}

BiasOffsets ReefwingLSM9DS1::calibrateAccelFIFO(bool save) {
  BiasOffsets bias;

  //  Turn on FIFO and set sample threshold to 32
  enableFIFO(true);
  setFIFOMode(FIFO_THS, 0x1F);

  //  Wait for FIFO to fill
  while (getFIFOSampleNumber() < 0x1F) {
    yield();
  }

  //  Average 32 zero-rate (bias offset) samples
  for (int i = 0; i < 32; i++) {
    RawData acc = readAccelRaw();

    bias.x += acc.rx;
    bias.y += acc.ry;
    bias.z += acc.rz - (int16_t)(1.0f/_aRes);
  }

  bias.x = bias.x / 32;
  bias.y = bias.y / 32;
  bias.z = bias.z / 32;

  if (save) {  setBiasOffset(ACCELEROMETER, bias);  }
  enableFIFO(false);
  setFIFOMode(BYPASS, 0x00);

  return bias;
}

BiasOffsets ReefwingLSM9DS1::calibrateMag(bool save) {
  BiasOffsets bias, min, max;

  for (int i = 0; i < 128; i++) {
    while (!magAvailable()) {
      yield();
    }

    RawData mag = readMagRaw();

    if (mag.rx > max.x) {  max.x = mag.rx;  }
    if (mag.rx < min.x) {  min.x = mag.rx;  }
    if (mag.ry > max.y) {  max.y = mag.ry;  }
    if (mag.ry < min.y) {  min.y = mag.ry;  }
    if (mag.rz > max.z) {  max.z = mag.rz;  }
    if (mag.rz < min.z) {  min.z = mag.rz;  }
  }

  bias.x = (max.x + min.x) / 2;
  bias.y = (max.y + min.y) / 2;
  bias.z = (max.z + min.z) / 2;

  if (save) {  setBiasOffset(MAGNETOMETER, bias);  }

  return bias;
}

void ReefwingLSM9DS1::setBiasOffset(SensorType sensor, BiasOffsets bias) {
  switch(sensor) {
    case SensorType::GYROSCOPE:
      _config.gyro.bias = bias;
      break;
    case SensorType::ACCELEROMETER:
      _config.accel.bias = bias;
      break;
    case SensorType::MAGNETOMETER:
      _config.mag.bias = bias;
      break;
  }
}

BiasOffsets ReefwingLSM9DS1::getBiasOffset(SensorType sensor) {
  switch(sensor) {
    case SensorType::GYROSCOPE:
      return _config.gyro.bias;
      break;
    case SensorType::ACCELEROMETER:
      return _config.accel.bias;
      break;
    case SensorType::MAGNETOMETER:
      return _config.mag.bias;
      break;
  }
}

SelfTestResults ReefwingLSM9DS1::selfTestGyroAccel() {
  SelfTestResults results;
  BiasOffsets gyro_noST, gyro_ST, accel_noST, accel_ST;

  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG10,   0x00); //  Disable self test
  gyro_noST = calibrateGyro(false);
  accel_noST = calibrateAccel(false);
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG10,   0x05); //  Enable gyro/accel self test
  gyro_ST = calibrateGyro(false);
  accel_ST = calibrateAccel(false);

  results.gyrodx = (gyro_ST.x - gyro_noST.x) * _gRes;
  results.gyrody = (gyro_ST.y - gyro_noST.y) * _gRes;
  results.gyrodz = (gyro_ST.z - gyro_noST.z) * _gRes;
  results.gyrodx = abs(results.gyrodx);
  results.gyrody = abs(results.gyrody);
  results.gyrodz = abs(results.gyrodz);

  results.accdx = 1000.0 * (accel_ST.x - accel_noST.x) * _aRes;
  results.accdy = 1000.0 * (accel_ST.y - accel_noST.y) * _aRes;
  results.accdz = 1000.0 * (accel_ST.z - accel_noST.z) * _aRes;
  results.accdx = abs(results.accdx);
  results.accdy = abs(results.accdy);
  results.accdz = abs(results.accdz);

  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG10,   0x00); //  Disable self test
  delay(200);

  return results;
}

BiasOffsets ReefwingLSM9DS1::averageMagOffsets() {
  int16_t mxr, myr, mzr;
  BiasOffsets bias;

  //  Wait until a sample is available
  while (!magAvailable()) {
    yield();
  }

  //  Flush the 1st sample
  readMagRaw();

  for (int i = 0; i < 5; i++) {
    while (!magAvailable()) {
      yield();
    }

    RawData mag = readMagRaw();

    bias.x += mag.rx;
    bias.y += mag.ry;
    bias.z += mag.rz;
  }

  //  Average the bias offsets
  bias.x = bias.x / 5;
  bias.y = bias.y / 5;
  bias.z = bias.z / 5;

  return bias;
}

MagTestResults ReefwingLSM9DS1::selfTestMag() {
  MagTestResults results;
  BiasOffsets mag_noST, mag_ST;

  //  Write 0x1C = 0b0001 1100 to CTRL_REG1_M
  //  ODR = 80 Hz, OM = Low Performance, FAST_ODR disabled, Self Test disabled
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x1C);

  //  Write 0x40 = 0b0100 0000 to CTRL_REG2_M
  //  FS = ± 12 gauss
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, 0x40);
  delay(20);

  //  Write 0x00 to CTRL_REG3_M
  //  Low Power Mode Disabled, Op Mode = Continuous
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00);
  delay(20);

  mag_noST = averageMagOffsets();

  //  Write 0x1D = 0b0001 1101 to CTRL_REG1_M
  //  ODR = 80 Hz, OM = Low Performance, FAST_ODR disabled, Self Test enabled
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x1D);
  delay(60);

  mag_ST = averageMagOffsets();

  //  Calculate absolute difference and scale to _mRes = ± 12 gauss
  results.magdx = (mag_ST.x - mag_noST.x) * SENSITIVITY_MAGNETOMETER_12;
  results.magdy = (mag_ST.y - mag_noST.y) * SENSITIVITY_MAGNETOMETER_12;
  results.magdz = (mag_ST.z - mag_noST.z) * SENSITIVITY_MAGNETOMETER_12;
  results.magdx = abs(results.magdx);
  results.magdy = abs(results.magdy);
  results.magdz = abs(results.magdz);

  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x1C);  //  Disable Self Test
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x03);  //  Power Down mode

  return results;
}

/******************************************************************
 *
 * LSM9DS1 Sensor Reading & Availability - 
 * 
 ******************************************************************/

int16_t ReefwingLSM9DS1::twosCompToInteger(uint16_t two_compliment_val) {
    // [0x0000; 0x7FFF] corresponds to [0; 32,767]
    // [0x8000; 0xFFFF] corresponds to [-32,768; -1]
    // int16_t has the range [-32,768; 32,767]

    uint16_t sign_mask = 0x8000;

    if ( (two_compliment_val & sign_mask) == 0 ) {
      // positive number - do nothing
      return two_compliment_val;
    } 
    else {
      //  if negative invert all bits, add one, and add sign
      return -(~two_compliment_val + 1);
    }
}

bool ReefwingLSM9DS1::gyroAvailable() {
	uint8_t STATUS_REG = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_STATUS_REG);
	
	return (STATUS_REG & 0x02);
}

bool ReefwingLSM9DS1::accelAvailable() {
	uint8_t STATUS_REG = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_STATUS_REG);
	
	return (STATUS_REG & 0x01);
}

bool ReefwingLSM9DS1::magAvailable() {
	uint8_t STATUS_REG_M = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_STATUS_REG_M);
	
	return (STATUS_REG_M & 0x08);
}

bool ReefwingLSM9DS1::tempAvailable() {
	uint8_t STATUS_REG = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_STATUS_REG);
	
	return (STATUS_REG & 0x04);
}

float ReefwingLSM9DS1::readTemp(TempScale scale) {
  uint8_t OUT_TEMP_L = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_OUT_TEMP_L);
  uint8_t OUT_TEMP_H = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_OUT_TEMP_H);

  uint16_t count = (OUT_TEMP_H << 8) | (OUT_TEMP_L & 0xff); 
  int16_t val = (int16_t)count;

  float result = ((float)val)/16.0f + _config.temp.offset;   // In Celsius

  switch(scale) {
    case TempScale::KELVIN:
      result += 273.15f;
      break;
    case TempScale::FAHRENHEIT:
      result = result * 1.8f + 32.0f;
      break;
    default:    //  Default scale is CELSIUS
      break;
  }

  return result;
}

ScaledData ReefwingLSM9DS1::readGyro() {
  RawData gyr;
  ScaledData result;

  //  Read the signed 16-bit RAW values
  gyr = readGyroRaw();

  //  Subtract the bias offsets
  gyr.rx -= _config.gyro.bias.x;
  gyr.ry -= _config.gyro.bias.y;
  gyr.rz -= _config.gyro.bias.z;

  //  Scale to DPS
  result.sx = gyr.rx * _gRes;
  result.sy = gyr.ry * _gRes;
  result.sz = gyr.rz * _gRes;

  return result;
}

RawData ReefwingLSM9DS1::readGyroRaw() {
  uint8_t regValue[6];
  RawData gyr;

  //  Read the six 8-bit gyro axis rate values
  readBytes(LSM9DS1AG_ADDRESS, LSM9DS1AG_OUT_X_L_G, 6, regValue);

  // Convert to the RAW signed 16-bit readings
  gyr.rx = (regValue[1] << 8) | regValue[0];
  gyr.ry = (regValue[3] << 8) | regValue[2];
  gyr.rz = (regValue[5] << 8) | regValue[4];

  return gyr;
}

ScaledData ReefwingLSM9DS1::readAccel() {
  RawData acc;
  ScaledData result;

  //  Read the signed 16-bit RAW values
  acc = readAccelRaw();

  //  Subtract the bias offsets
  acc.rx -= _config.accel.bias.x;
  acc.ry -= _config.accel.bias.y;
  acc.rz -= _config.accel.bias.z;

  //  Scale to G's
  result.sx = acc.rx * _aRes;
  result.sy = acc.ry * _aRes;
  result.sz = acc.rz * _aRes;

  return result;
}

RawData ReefwingLSM9DS1::readAccelRaw() {
  uint8_t regValue[6];
  RawData acc;

  //  Read the six 8-bit accelerometer axis values
  readBytes(LSM9DS1AG_ADDRESS, LSM9DS1AG_OUT_X_L_XL, 6, regValue);

  // Convert to the RAW signed 16-bit readings
  acc.rx = (regValue[1] << 8) | regValue[0];
  acc.ry = (regValue[3] << 8) | regValue[2];
  acc.rz = (regValue[5] << 8) | regValue[4];

  return acc;
}

ScaledData ReefwingLSM9DS1::readMag() {
  RawData mag;
  ScaledData result;

  //  Read the signed 16-bit RAW values
  mag = readMagRaw();

  //  Subtract the bias offsets
  mag.rx -= _config.mag.bias.x;
  mag.ry -= _config.mag.bias.y;
  mag.rz -= _config.mag.bias.z;

  //  Scale to Gauss
  result.sx = mag.rx * _mRes;
  result.sy = mag.ry * _mRes;
  result.sz = mag.rz * _mRes;

  return result;
}

RawData ReefwingLSM9DS1::readMagRaw() {
  uint8_t regValue[6];
  RawData mag;

  //  Read the six 8-bit magnetometer axis values
  readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, regValue);

  // Convert to the RAW signed 16-bit readings
  mag.rx = (regValue[1] << 8) | regValue[0];
  mag.ry = (regValue[3] << 8) | regValue[2];
  mag.rz = (regValue[5] << 8) | regValue[4];

  return mag;
}

/******************************************************************
 *
 * I2C Read/Write methods - 
 * 
 ******************************************************************/

uint8_t ReefwingLSM9DS1::readByte(uint8_t address, uint8_t regAddress) {
  _wire->beginTransmission(address);
  _wire->write(regAddress);
  _wire->endTransmission(false);
  _wire->requestFrom(address, 1);

  return _wire->read();
}

void ReefwingLSM9DS1::readBytes(uint8_t address, uint8_t regAddress, uint8_t numBytes, uint8_t *dest) {  
  uint8_t i = 0;

  _wire->beginTransmission(address);   
  _wire->write(regAddress);            
  _wire->endTransmission(false);       
  _wire->requestFrom(address, numBytes);  
  
  while (_wire->available()) {
    dest[i++] = _wire->read(); }         
}

void ReefwingLSM9DS1::writeByte(uint8_t address, uint8_t regAddress, uint8_t data) {
  _wire->beginTransmission(address);
  _wire->write(regAddress);
  _wire->write(data);
  _wire->endTransmission();
}