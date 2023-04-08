/******************************************************************
  @file       LSM9DS1_Types.h
  @brief      I2C, Types and Enumerations for the LSM9DS1 9-axis IMU
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     1.0.0
  Date:        20/03/23

  1.0.0     Original Release.       20/03/23

******************************************************************/

/******************************************************************
 *
 * Select I2C Device Bus based on device - 
 * 
 ******************************************************************/

#ifdef ARDUINO_ARDUINO_NANO33BLE
    #define WIRE Wire1
#else
    #define WIRE Wire
#endif

/******************************************************************
 *
 * I2C Device Addresses - 
 * 
 ******************************************************************/

#define LSM9DS1AG_ADDRESS 0x6B  //  Address of accelerometer & gyroscope
#define LSM9DS1M_ADDRESS  0x1E  //  Address of magnetometer 

#define HTS221_ADDRESS    0x5F  //  Nano 33 BLE Sense Rev 1 Sensor - temp/humidity
#define HS3003_ADDRESS    0x44  //  Nano 33 BLE Sense Rev 2 Sensor - temp/humidity

/******************************************************************
 *
 * Configuration Constants - Sensor Sensitivity ref: LSM9DS1 Data
 *                           Sheet, Table 3, Page 12.
 * 
 ******************************************************************/

#define FLUSH_SAMPLES   20      //  Number of samples flushed after reset

#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
#define SENSITIVITY_MAGNETOMETER_4   0.00014
#define SENSITIVITY_MAGNETOMETER_8   0.00029
#define SENSITIVITY_MAGNETOMETER_12  0.00043
#define SENSITIVITY_MAGNETOMETER_16  0.00058

/******************************************************************
 *
 * ENUM Definitions - 
 * 
 ******************************************************************/

enum class TempScale {
  CELSIUS = 0,
  KELVIN,
  FAHRENHEIT
};

//  Available Gyroscope/Accelerometer Operating Modes
//  Ref - LSM9DS1 Data Sheet, Figure 5, page 19
enum GyroAccelOpModes {
  POWERED_DOWN = 0,
  ACCELEROMETER,
  NORMAL,
  LOW_POWER
};

//  Available Magnetometer Operating Modes
//  Ref - LSM9DS1 Data Sheet, Table 110, Page 63
//  Set in CTRL_REG1_M (x and y axis) & CTRL_REG4_M (z axis)
//  For LOW_POWER mode, the Low Power (LP) bit = 1, 
//  in CTRL_REG3_M, and DO = 0.625 Hz
enum MagOpModes {
  LOW = 0,        //  OM = 00
  MEDIUM,         //  OM = 01
  HIGH,           //  OM = 10
  ULTRA           //  OM = 11
};

//  Available Magnetometer Sample Modes
//  Ref - LSM9DS1 Data Sheet, Table 117, Page 64
//  Set in CTRL_REG3_M
enum MagSampleModes {
  CONTINUOUS = 0, //  MD = 00
  SINGLE_SHOT,    //  MD = 01
  POWERED_DOWN    //  MD = 10 or 11
};

//  Allowable accelerometer full scale settings
//  Ref - LSM9DS1 Data Sheet, Table 67, Page 52
enum AccelScale {  
  FS_XL_2G = 0,      //  FS_XL = 00
  FS_XL_16G,         //  FS_XL = 01
  FS_XL_4G,          //  FS_XL = 10
  FS_XL_8G           //  FS_XL = 11
};

//  Allowable gyroscope full scale settings
//  Ref - LSM9DS1 Data Sheet, Table 45, Page 45
enum GyroScale {  
  FS_245DPS = 0,    //  FS_G = 00
  FS_500DPS,        //  FS_G = 01
  FS_2000DPS        //  FS_G = 11
};

//  Allowable magnetometer full scale settings
//  Ref - LSM9DS1 Data Sheet, Table 114, Page 64
enum MagScale {  
  FS_4G = 0,        //  FS = 00
  FS_8G,            //  FS = 01
  FS_12G,           //  FS = 10
  FS_16G            //  FS = 11
};

//  Allowable accelerometer sample rates
//  Ref - LSM9DS1 Data Sheet, Table 68, Page 52
enum AccelODR {  
  ODR_PowerDown = 0,  //  ODR_XL = 000
  ODR_10Hz,           //  ODR_XL = 001
  ODR_50Hz,           //  ODR_XL = 010
  ODR_119Hz,          //  ODR_XL = 011
  ODR_238Hz,          //  ODR_XL = 100
  ODR_476Hz,          //  ODR_XL = 101
  ODR_952Hz           //  ODR_XL = 110
};

//  Allowable gyroscope sample rates (ODR)
//  Ref - LSM9DS1 Data Sheet, Tables 45 & 46, Page 45
enum GyroODR {  
  ODR_PowerDown = 0,  //  ODR_G = 000
  ODR_14_9Hz,         //  ODR_G = 001
  ODR_59_5Hz,         //  ODR_G = 010
  ODR_119Hz,          //  ODR_G = 011
  ODR_238Hz,          //  ODR_G = 100
  ODR_476Hz,          //  ODR_G = 101
  ODR_952Hz           //  ODR_G = 110
};

//  Allowable magnetometer sample rates
//  Ref - LSM9DS1 Data Sheet, Table 111, Page 63
enum MagODR {  
  ODR_0_625Hz = 0,    //  DO = 000, FAST_ODR = 0
  ODR_1_25Hz,         //  DO = 001
  ODR_2_5Hz,          //  DO = 010
  ODR_5Hz,            //  DO = 011
  ODR_10Hz,           //  DO = 100
  ODR_20Hz,           //  DO = 101
  ODR_40Hz,           //  DO = 110
  ODR_80Hz,           //  DO = 111
  ODR_155Hz,          //  OM = 11,  FAST_ODR = 1
  ODR_300Hz,          //  OM = 10
  ODR_560Hz,          //  OM = 01
  ODR_1000Hz          //  OM = 00
};

//  Allowable accelerometer anti-aliasing filter bandwidths
//  Available when BW_SCAL_ODR = 1 in CTRL_REG6_XL
//  Ref - LSM9DS1 Data Sheet, Table 68, Page 52
enum AccelBW {  
  BW_408Hz = 0, //  BW_XL = 00
  BW_211Hz,     //  BW_XL = 01
  BW_105Hz,     //  BW_XL = 10
  BW_50Hz       //  BW_XL = 11
};

//  Allowable gyroscope bandwidths
//  Ref - LSM9DS1 Data Sheet, Tables 45, 46 & 47, Pages 45 & 46
//  BW_G is only applicable if Low Pass Filter 2 (LPF2) is active
enum GyroBW {   
  LOW = 0,      // BW_G = 00, 14 Hz at ODR = 238 Hz,  33 Hz at ODR = 952 Hz
  MEDIUM,       // BW_G = 01, 29 Hz at ODR = 238 Hz,  40 Hz at ODR = 952 Hz
  HIGH,         // BW_G = 10, 63 Hz at ODR = 238 Hz,  58 Hz at ODR = 952 Hz
  MAXIMUM       // BW_G = 11, 78 Hz at ODR = 238 Hz, 100 Hz at ODR = 952 Hz
};

//  Reverse Gyro Reading Sign
//  Ref - LSM9DS1 Data Sheet, Tables 53 & 54, Page 48
//        Applied to the ORIENT_CFG_G Register
enum GyroReverseSign {
  NONE = 0,
  SIGN_Z,       //  SignXYZ_G = 001
  SIGN_Y,       //  SignXYZ_G = 010
  SIGN_Y_Z,     //  SignXYZ_G = 011
  SIGN_X,       //  SignXYZ_G = 100
  SIGN_X_Z,     //  SignXYZ_G = 101
  SIGN_X_Y,     //  SignXYZ_G = 110
  SIGN_X_Y_Z    //  SignXYZ_G = 111
};

//  Gyro Orientation for pitch, roll, and yaw (PRY),
//  as referenced to the X, Y, and Z axis
//  Ref - LSM9DS1 Data Sheet, Tables 53 & 54, Page 48
//        Applied to the ORIENT_CFG_G Register
enum GyroOrient {
  PRY_XYZ = 0,  //  ORIENT = 000
  PRY_XZY,      //  ORIENT = 001
  PRY_YXZ,      //  ORIENT = 010
  PRY_YZX,      //  ORIENT = 011
  PRY_ZXY,      //  ORIENT = 100
  PRY_ZYX       //  ORIENT = 101
};

//  FIFO Memory for Gyroscope and Accelerometer Data
//  Ref- LSM9DS1 Data Sheet, Table 84, Page 56
//       Applied to the FIFO_CTRL Register
enum FIFOMode {
  BYPASS = 0,                 //  FMODE = 000, default
  FIFO_THS = 1,               //  FMODE = 001
  CONTINUOUS_TO_FIFO = 3,     //  FMODE = 011
  BYPASS_TO_CONTINUOUS = 4,   //  FMODE = 100
  FIFO_CONTINUOUS = 6         //  FMODE = 110
};

enum SensorType {
  GYROSCOPE = 0,
  ACCELEROMETER,
  MAGNETOMETER
};

/******************************************************************
 *
 * Struct Definitions - 
 * 
 ******************************************************************/

struct GyroConfig {
  bool powerDown = true;
  bool sleepEnabled = false;
  bool lowPowerEnabled = false;

  GyroScale scale;
  GyroODR sampleRate;
  GyroBW bandwidth;
  GyroOrient orientation;
  GyroReverseSign reverseSign;
  BiasOffsets bias;
};

struct AccelConfig {
  bool powerDown = true;
  bool autoBandwidthEnable = true;

  AccelScale scale;
  AccelODR sampleRate;
  AccelBW bandwidth;
  BiasOffsets bias;
};

struct MagConfig {
  bool fastODR = false;
  bool temperatureCompensated = false;

  MagOpModes opMode;
  MagSampleModes sampleMode;

  MagScale scale;
  MagODR sampleRate;
  BiasOffsets bias;
};

struct TempConfig {
  float offset = 25.0f;   //  Ref: LSM9DS1 Data Sheet, Table 5, Note 3
};

struct Configuration {
  bool gyroAccelFIFOEnabled = false;
  uint8_t fifoThreshold = 0;
  FIFOMode fifoMode = BYPASS;
  GyroAccelOpModes gyroAccelOpMode;

  GyroConfig gyro;
  AccelConfig accel;
  MagConfig mag;
  TempConfig temp;
};

struct BiasOffsets {
  int16_t x, y, z;
};

struct SelfTestResults {
  float gyrodx;
  float gyrody;
  float gyrodz;
  float accdx;
  float accdy;
  float accdz;
};

struct MagTestResults {
  float magdx;
  float magdy;
  float magdz;
}

struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};
