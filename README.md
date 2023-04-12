![version](https://img.shields.io/github/v/tag/Reefwing-Software/Reefwing-LSM9DS1) ![license](https://img.shields.io/badge/license-MIT-green) ![release](https://img.shields.io/github/release-date/Reefwing-Software/Reefwing-LSM9DS1?color="red") ![open source](https://badgen.net/badge/open/source/blue?icon=github)

# Reefwing LSM9DS1

 An Arduino Library for the LSM9DS1 9-axis IMU, which targets the Arduino Nano 33 BLE and the Arduino Nano 33 BLE Sense Revision 1. This library differs from the official Arduino LSM9DS1 library in the following ways:

 - Where it makes sense, we have emphasized readability over code size. Configuring the IMU requires a lot of register operations, which can be done very concisely, but it then becomes harder to understand what is happening. Configuration will normally only be done once, at the start of the program, so this should not impact loop speed.
- We include the option to set the `LOW_POWER` gyro/accelerometer operation mode. You can also put the gyro into `SLEEP` mode.
- It is possible to enable/disable accelerometer auto bandwidth and set this manually if required.
- You can set any of the five FIFO modes (Arduino library includes only continuous and single shot). A method is provided to return the number of samples currently stored in FIFO memory.
- The Arduino Library allows you to calibrate both the slope and the bias offset. Our library currently only calibrates the bias offset.
- You can reverse the gyro sign on any axis and change the sensor orientation using `reverseGyroSign()` and `setGyroOrientation()`.
- Enable FastODR for the magnetometer and you can use the higher sample rates (155, 300, 560, & 1000 Hz) associated with this. You can also enable temperature compensation for the magnetometer.
The magnetometer operating modes (`LOW`, `MEDIUM`, `HIGH` & `ULTRA`) can be set in addition to the sampling modes (`CONTINUOUS`, `SINGLE`, & `IDLE`).
- Gyro chip temperature and its availability is provided.
- Self Test methods are provided for all three sensors. The gyro/accel version should be considered experimental as ST do not document this function. The magnetometer self test feature is well documented.
It is designed to be compatible with the new and improved Reefwing AHRS Library.

## The LSM9DS1 IMU

The LSM9DS1 is manufactured by STMicroelectronics (now known as ST). It has a 3D digital linear acceleration sensor, a 3D digital angular rate sensor (gyroscope), and a 3D digital magnetic sensor. It includes SPI and I2C (standard 100 kHz and fast mode 400 kHz) serial interfaces but the Arduino Nano 33 BLE uses I2C. Full details of this chip are available in the [LSM9DS1 data sheet](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf). On the LSM9DS1, magnetometer, accelerometer and gyroscope sensing can be enabled or set to power-down mode, for smart power management.

A gyroscope is a device that can measure the orientation and angular velocity of an object. Gyroscopes are more advanced than accelerometers, in that they can measure the tilt and lateral orientation, whereas an accelerometer can only measure its linear motion. Gyroscope sensors are also called "Angular Rate Sensors" or "Angular Velocity Sensors". Angular velocity (measured in degrees per second - DPS) is the change in the rotational angle of the object per unit of time.

An accelerometer is an electromechanical device used to measure acceleration forces. Such forces may be static, like the continuous force of gravity or, as with a drone accelerating, dynamic to sense movement or vibrations. Changes in the acceleration force measured in the 3-axis directions can be used to determine the orientation of a drone. Accelerometer sensors are insensitive to rotation about the earth's gravitational field vector.

The LSM9DS1 data sheet (included in the data sheet folder of this library) shows the positive x, y, and z axes on the accelerometer sensor package. These are defined so that a linear acceleration aligned in the direction of these axes will give a positive accelerometer output. A gravitational field component aligned along these axes directions will result in a negative reading on the accelerometer. Changes in orientation are described by rotations in roll φ, pitch θ and yaw ψ about the x, y and z axes respectively.

An important point is that the output of the accelerometer is sinusoidal as it is rotated through gravity. The acceleration detected by the x-axis is proportional to the sine of the angle of inclination. The y-axis acceleration, due to orthogonality (i.e., the 2 axes are at right angles), is proportional to the cosine of the angle of inclination.

A magnetometer is an instrument used for measuring magnetic forces, and in our context, the earth's magnetism. The Earth's magnetic field is a 3-dimensional vector that, like gravity, can be used to determine long-term orientation. The typical magnitude of the Earth's magnetic field is between 20 µT and 70 µT.

## Register Mapping

The LSM9DS1 device contains a set of 8-bit registers which are used to control its behavior and to retrieve IMU and temperature data. The register address, made up of 7 bits, is used to identify them and to read/write the data through the serial interface. The registers that we use are shown in Tables 21 and 22 of the LSM9DS1 data sheet.

## Who Am I?

The LSM9DS1 gyroscope/accelerometer and magnetometer have different I²C addresses on the bus.

``` c++
#define LSM9DS1AG_ADDRESS 0x6B  // Address of accelerometer & gyroscope
#define LSM9DS1M_ADDRESS  0x1E  // Address of magnetometer
```

Most chips include a register which we can read to positively identify the device on the I²C bus. For the LSM9DS1, there are two `WHO_AM_I` registers, one for the gyroscope/accelerometer , and one for the magnetometer, both are at register address `0x0F`. The value contained in the gyroscope/acclerometer register is `0x68 = 0110 1000`. The value of the magnetometer `WHO_AM_I` register is `0x3D = 0011 1101`.

Our library has a function which reads the two `WHO_AM_I` registers and another called `connected()`, which confirms that the values in the registers are as expected.

``` c++
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
```

## IMU Reset

To reset the gyroscope/accelerometer we write to bit `0` of control register 8, `CTRL_REG8` (`0x22`) - Figure 1.


|   7  |  6  |     5     |   4   |  3  | 2          | 1   | 0        |
|:----:|:---:|:---------:|:-----:|:---:|------------|-----|----------|
| BOOT | BDU | H_LACTIVE | PP_OD | SIM | IF_ADD_INC | BLE | SW_RESET |

*Figure 1. Control Register 8*

The explanation of the bits in Control Register 8 are shown in Table 73 of the LSM9DS1 Data Sheet.

As bit `IF_ADD_INC` of control register 8 defaults to 1, and the other bits default to zero, we will write `0000 0101 = 0x05` to reset the gyro/accelerometer.

Similarly, for the magnetometer, we write to control register 2 `CTRL_REG2_M` (`0x21`) - Figure 2, and set bits 2 and 3 (i.e., `0000 1100 = 0x0C`).


| 7 |  6  |  5  | 4 |    3   |     2    | 1 | 0 |
|:-:|:---:|:---:|:-:|:------:|:--------:|:-:|:-:|
| 0 | FS1 | FS0 | 0 | REBOOT | SOFT_RST | 0 | 0 |

*Figure 2. Control Register 2 M*

The two `FS` bits are used to select the full-scale resolution of the magnetometer as displayed in Figure 3. The default is `00`.

| **FS1** | **FS0** | **Full Scale** |
|:-------:|:-------:|:--------------:|
|    0    |    0    |    ± 4 gauss   |
|    0    |    1    |    ± 8 gauss   |
|    1    |    0    |   ± 12 gauss   |
|    1    |    1    |   ± 16 gauss   |

*Figure 3. Magnetometer Full-scale Selection*

The `reset()` method for our library is shown below. The reset method is called from `begin()`.

``` c++
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
```
Tables 11 and 12 in the LSM9DS1 Data Sheet, specify how many samples should be discarded after reset, to allow for the sensor turn-on times. We discard 20 readings, to be safe. This is also handled in `begin()`.

## Gyroscope/Accelerometer Operating Modes

In the LSM9DS1, the accelerometer and gyroscope has four operating modes available: 

- Powered Down: Accelerometer and gyroscope are both off.
- Accelerometer Only: The accelerometer is active and the gyroscope powered down.
- Normal Mode: Both accelerometer and gyroscope sensors are active and running at the same sample rate, also called the Output Data Rate = `ODR`.
- Low Power Mode: Reduces the current consumption of the accelerometer and gyroscope, but is only available for the lower sample rates (`ODR` = 14.9, 59.5, or 119 Hz - Figure 4). In addition to selecting these rates, the low power mode (`LP_mode`) bit in `CTRL_REG3` must be set to enable low power mode - Figure 5.

| ODR (Hz) | Power Mode | Current (mA) |
|:--------:|:----------:|:------------:|
|   14.9   |     LOW    |      1.9     |
|   59.5   |     LOW    |      2.4     |
|    119   |     LOW    |      3.1     |
|    238   |   NORMAL   |      4.3     |
|    476   |   NORMAL   |      4.3     |
|    952   |   NORMAL   |      4.3     |

*Figure 4. Operating Mode, Data Rate and Current Consumption* 

The configuration of angular rate sensor Control Register 3 (`CTRL_REG3_G`, `0x12`) is illustrated in Figure 5.

|    7    |   6   | 5 | 4 |    3    |    2    |    1    |    0    |
|:-------:|:-----:|:-:|:-:|:-------:|:-------:|:-------:|:-------:|
| LP_mode | HP_EN | 0 | 0 | HPCF3_G | HPCF2_G | HPCF1_G | HPCF0_G |

*Figure 5. Gyro Control Register 3 - Enabling Low Power Mode*

To set the gyro/accelerometer to normal mode, we write to the angular rate sensor Control Register 1, `CTRL_REG1_G` (`0x10`) - Figure 6.

|    7   |    6   |    5   |   4   |   3   | 2 |   1   |   0   |
|:------:|:------:|:------:|:-----:|:-----:|:-:|:-----:|:-----:|
| ODR_G2 | ODR_G1 | ODR_G0 | FS_G1 | FS_G0 | 0 | BW_G1 | BW_G0 |

*Figure 6. Angular Rate Sensor Control Register 1, CTRL_REG1_G (0x10)*

We will adjust the first 5 bits of this control register to adjust the mode, sample rate (`ODR_G`) and resolution (full-scale selection, `FS_G`). Figure 7 shows the sample rates available, the default value is `000`.

| ODR_G2 | ODR_G1 | ODR_G0 |  ODR (Hz)  | LPF Cutoff (Hz) |
|:------:|:------:|:------:|:----------:|:---------------:|
|    0   |    0   |    0   | Power-down |       N/A       |
|    0   |    0   |    1   |    14.9    |        5        |
|    0   |    1   |    0   |    59.5    |        19       |
|    0   |    1   |    1   |     119    |        38       |
|    1   |    0   |    0   |     238    |        76       |
|    1   |    0   |    1   |     476    |       100       |
|    1   |    1   |    0   |     952    |       100       |
|    1   |    1   |    1   |     N/A    |       N/A       |

*Figure 7. Gyro/Accelerometer Mode & Sample Rate Options [Derived from the LSM9DS1 Data Sheet - Table 46]*

If you are only using the accelerometer, the equivalent control register is `CTRL_REG6_XL` (Figure 8).

|    7    |    6    |    5    |    4   |    3   |      2      |    1   |    0   |
|:-------:|:-------:|:-------:|:------:|:------:|:-----------:|:------:|:------:|
| ODR_XL2 | ODR_XL1 | ODR_XL0 | FS1_XL | FS0_XL | BW_SCAL_ODR | BW_XL1 | BW_XL0 |

*Figure 8. Linear acceleration sensor Control Register 6*

The lower sample rate options in `ACCELEROMETER` mode are different to `NORMAL mode`. This can be seen by comparing, Figure 7 and Figure 9. The accelerometer full scale (`FS_XL`) options are:

- `FS_XL = 00` = ±2 g (default)
- `FS_XL = 01` = ±16 g
- `FS_XL = 10` = ±4 g
- `FS_XL = 11` = ±8 g

| ODR_XL2 | ODR_XL1 | ODR_XL0 |  ODR (Hz)  |
|:-------:|:-------:|:-------:|:----------:|
|    0    |    0    |    0    | Power-down |
|    0    |    0    |    1    |     10     |
|    0    |    1    |    0    |     50     |
|    0    |    1    |    1    |     119    |
|    1    |    0    |    0    |     238    |
|    1    |    0    |    1    |     476    |
|    1    |    1    |    0    |     952    |
|    1    |    1    |    1    |     N/A    |

*Figure 9. ODR register setting (accelerometer only mode) [LSM9DS1 Data Sheet - Table 68]*

Initially we thought that we needed a method to set the operating mode, but a better approach is to allow the operating mode to emerge as part of setting the sample rate and low power enable bit.

In summary, to set the accelerometer/gyroscope operating mode we need to write to 2–3 registers, depending on the mode. We will talk more about this when we explain setting the gyroscope and accelerometer configuration. Each register includes control bits for various settings and it makes sense to modify bits on a common register at the same time if possible.

## Magnetometer Operating Modes

Things are a bit different with the magnetometer, it has four operating modes and three sampling modes, controlled by different registers. The four operating modes (OM) are:

- Low Performance (default), `OM = 00`; 
- Medium Performance, `OM = 01`;
- High Performance, `OM = 10`; and
- Ultra-high Performance, `OM = 11`.

These are set using `CTRL_REG1_M` (Figure 10), for the x and y axis, and `CTRL_REG4_M` (Figure 11) for the z axis. For the output of the magnetic data to be compensated for temperature, the `TEMP_COMP` bit in `CTRL_REG1_M` (`0x20`) must be set to `1`. You can use the `enableMagTempComp()` method to turn this off or on, the default is off.

|     7     |  6  |  5  |  4  |  3  |  2  |     1    |  0 |
|:---------:|:---:|:---:|:---:|:---:|:---:|:--------:|:--:|
| TEMP_COMP | OM1 | OM0 | DO2 | DO1 | DO0 | FAST_ODR | ST |

*Figure 10. Magnetometer Control Register 1, CTRL_REG1_M*

| 7 | 6 | 5 | 4 |   3  |   2  |  1  | 0 |
|:-:|:-:|:-:|:-:|:----:|:----:|:---:|:-:|
| 0 | 0 | 0 | 0 | OMZ1 | OMZ0 | BLE | 0 |

*Figure 11. Magnetometer Control Register 4, CTRL_REG4_M*

If `FAST_ODR` in `CTRL_REG1_M` is disabled, then the sample rate (i.e., `ODR`) is set by `DO`, in Figure 10. The sample rates for `FAST_ODR = 0`, are shown in Figure 12.

| DO2 | DO1 | DO0 | ODR (Hz) |
|:---:|:---:|:---:|:--------:|
|  0  |  0  |  0  |   0.625  |
|  0  |  0  |  1  |   1.25   |
|  0  |  1  |  0  |    2.5   |
|  0  |  1  |  1  |     5    |
|  1  |  0  |  0  |    10    |
|  1  |  0  |  1  |    20    |
|  1  |  1  |  0  |    40    |
|  1  |  1  |  1  |    80    |

*Figure 12. Magnetometer ODR Configuration (FAST_ODR Disabled)*

If `FAST_ODR = 1`, the available sample rates are set by the operating modes (`OM`), Figure 13, the `DO` bits have no effect. The operating modes of the magnetometer are differentiated by performance. So in Low Performance mode, the number of samples averaged is less so the `ODR` is higher. In ultra high performance mode, the number of samples averaged is high and the resulting `ODR` is lower.

| Performance | OM1 | OM0 | ODR (Hz) |
|:-----------:|:---:|:---:|:--------:|
|     LOW     |  0  |  0  |   1000   |
|    MEDIUM   |  0  |  1  |    560   |
|     HIGH    |  1  |  0  |    300   |
|    ULTRA    |  1  |  1  |    155   |

*Figure 13. Magnetometer ODR Configuration (FAST_ODR Enabled)*

The magnetometer has three sampling modes (MD) available: 

- Continuous-conversion mode, `MD = 00`;
- Single-conversion mode, `MD = 01`; and
- Powered down or Idle (default), `MD = 10` or `MD = 11`. 

During single-measurement mode, the device performs a single measurement and writes the measured data in the data output register. After the measurement has been completed and the output data registers are updated, the device is reset to idle mode: the Mode register is consequently changed to idle mode (`MD = 10`).

During continuous-measurement mode, the device continuously performs measurements and writes measured data in the data output registers. In order to reduce average power consumption, the device is configured in a state similar to idle mode (called `TOFF`) between two consecutive measurements.

The modes are assigned via `CTRL_REG3_M` (Figure 14). Use the library method, `setSampleMode()`, to set the required mode. The library default is `CONTINUOUS`.

|      7      | 6 |  5 | 4 | 3 |  2  |  1  |  0  |
|:-----------:|:-:|:--:|:-:|:-:|:---:|:---:|:---:|
| I2C_DISABLE | 0 | LP | 0 | 0 | SIM | MD1 | MD0 |

*Figure 14. Magnetometer Control Register 3, CTRL_REG3_M*

## IMU Configuration

To assist with assigning and storing the various settings available for the LSM9DS1, we have included the `ImuConfig` structure. This is not directly accessed, but available via the class methods.

``` c++
struct Configuration {
  GyroAccelOpModes gyroAccelOpMode;
  GyroConfig gyro;
  AccelConfig accel;
  MagConfig mag;
};
```

The default configuration settings are set and loaded in the `begin()` method. The IMU starts taking sensor readings when `start()` is called. If you want to change the IMU configuration, do this after `begin()` but before calling `start()`.

``` c++
//  Default configuration
  enableLowPower(false);
  enableAccelAutoBandwidth(true);
  setSampleMode(CONTINUOUS);
  setGyroScale(FS_2000DPS);
  setAccelScale(FS_XL_2G);
  setMagScale(FS_4G);
  setGyroBandwidth(LOWEST);
  setGyroODR(GODR_119Hz);    //  NORMAL Op Mode, gyro rate = accel rate
  setMagODR(MODR_10Hz);
```

## Configuration - Scale

The gyroscope scale is set using the angular rate sensor Control Register 1, `CTRL_REG1_G`. The full scale selection (`FS_G`) options for the gyroscope are:

- `FS = 00` = 245 dps (default)
- `FS = 01` = 500 dps
- `FS = 10` = N/A
- `FS = 11` = 2000 dps

The default `FS_G = 00`. 

Our approach, is to copy `CTRL_REG1_G`, mask out the two `FS_G` bits (i.e., `FS_G = 00`), replace these two bits with the appropriate encoding, and finally write the new register value back to `CTRL_REG1_G`. The `setGyroScale()` method is shown below.

``` c++
void ReefwingLSM9DS1::setGyroScale(GyroScale scale) {
  uint8_t CTRL_REG1_G = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G);

  //  Clear the FS bits (3 & 4), maintain the rest
  //  0xE7 = 0b11100111, FS = 00, scale = 245 DPS
  CTRL_REG1_G &= 0xE7;

  switch(scale) {
    case GyroScale::FS_500DPS:
      CTRL_REG1_G |= (0x01 << 3);
      _config.gyro.scale = GyroScale::FS_500DPS;
      break;
    case GyroScale::FS_2000DPS:
      CTRL_REG1_G |= (0x03 << 3);
      _config.gyro.scale = GyroScale::FS_2000DPS;
      break;
    default:  //  Default scale is 245 DPS
      _config.gyro.scale = GyroScale::FS_245DPS;
      break;
  }

  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G, CTRL_REG1_G);
}
```

We do something similar for `setAccelScale()` and `setMagScale()`, but with different registers.

## Configuration - Sample Rate (ODR)

Figure 7 shows the sample rates or Output Data Rates (`ODR`) available for the gyroscope. We use the same `CTRL_REG1_G` to assign a rate. Again, we copy `CTRL_REG1_G`, mask out the three `ODR_G` bits (i.e., `ODR_G = 000`), replace these three bits with the appropriate encoding, and finally write the new register value back to `CTRL_REG1_G`. The `setGyroScale()` method is produced below.

``` c++
void ReefwingLSM9DS1::setGyroODR(GyroODR rate) {
  uint8_t CTRL_REG1_G = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G);

  //  Clear the ODR_G bits (7, 6 & 5), maintain the rest
  //  0x1F = 0b00011111, ODR_G = 000, mode = power-down
  CTRL_REG1_G &= 0x1F;

  _config.gyro.powerDown = false;
  _config.gyroAccelOpMode = NORMAL;

  switch(rate) {
    case GyroODR::ODR_14_9Hz:   //  ODR_G = 001
      CTRL_REG1_G |= (0x01 << 5);
      if (_config.gyro.lowPowerEnabled) {
        _config.gyroAccelOpMode = LOW_POWER;
      }
      break;
    case GyroODR::ODR_59_5Hz:   //  ODR_G = 010
      CTRL_REG1_G |= (0x02 << 5);
      if (_config.gyro.lowPowerEnabled) {
        _config.gyroAccelOpMode = LOW_POWER;
      }
      break;
    case GyroODR::ODR_119Hz:   //  ODR_G = 011
      CTRL_REG1_G |= (0x03 << 5);
      if (_config.gyro.lowPowerEnabled) {
        _config.gyroAccelOpMode = LOW_POWER;
      }
      break;
    case GyroODR::ODR_238Hz:   //  ODR_G = 100
      CTRL_REG1_G |= (0x04 << 5);
      break;
    case GyroODR::ODR_476Hz:   //  ODR_G = 101
      CTRL_REG1_G |= (0x05 << 5);
      break;
    case GyroODR::ODR_952Hz:   //  ODR_G = 110
      CTRL_REG1_G |= (0x06 << 5);
      break;
    default:  //  ODR_G = 000, default rate is power-down
      _config.gyro.powerDown = true;
      _config.gyroAccelOpMode = ACCELEROMETER;
      if (_config.accel.powerDown) {
        _config.gyroAccelOpMode = POWERED_DOWN;
      }
      break;
  }

  _config.gyro.sampleRate = rate;
  if (_config.gyroAccelOpMode != ACCELEROMETER) {
    _config.accel.sampleRate = rate;
  }
  writeByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G, CTRL_REG1_G);
}
```

As for the scale, we have similar sample rate setting methods for the accelerometer and magnetometer (`setAccelODR()` and `setMagODR()`). The magnetometer method is a bit different, because it has to handle `FAST_ODR` for rates greater than 8o Hz, as explained previously. This part of the `setMagODR()` method, is shown below.

``` c++
void ReefwingLSM9DS1::setMagODR(MagODR rate) {

  if (rate > ODR_80Hz) {
    enableFastODR(true);

    switch(rate) {
      case MagODR::ODR_155Hz:       //  OM = 11, mode = ULTRA
        setMagOperatingMode(ULTRA);
        break;
      case MagODR::ODR_300Hz:       //  OM = 10, mode = HIGH
        setMagOperatingMode(HIGH);
        break;
      case MagODR::ODR_560Hz:       //  OM = 01, mode = MEDIUM
        setMagOperatingMode(MEDIUM);
        break;
      case MagODR::ODR_1000Hz:      //  OM = 00 (OM default), mode = LOW
        setMagOperatingMode(LOW);
        break;
    }
  } 
  else {
    enableFastODR(false);
    
    ...
```

For the faster rates, enabling `FAST_ODR` and setting the operating mode is handled automatically.

## Configuration - Bandwidth

Unless you enable the High Pass Filter (HPF) in the gyroscope signal path, the bandwidth (actually, the cutoff of the low pass filter - `LPF1`) is automatically set depending on the output data rate (`ODR`) that you select (Figure 15).

If the high pass filter is enabled using `CTRL_REG3_G`, then the bandwidth is the difference between the selected highpass and low pass filters. This frequency is provided in Table 47 of the LSM9DS1 Data Sheet. If you enable the `HPF`, you can set its cutoff frequency using the same register.

| ODR_G2 | ODR_G1 | ODR_G0 |  ODR (Hz)  | LPF1 Cutoff (Hz) |
|:------:|:------:|:------:|:----------:|:----------------:|
|    0   |    0   |    0   | Power-down |        N/A       |
|    0   |    0   |    1   |     10     |         5        |
|    0   |    1   |    0   |     50     |        19        |
|    0   |    1   |    1   |     119    |        38        |
|    1   |    0   |    0   |     238    |        76        |
|    1   |    0   |    1   |     476    |        100       |
|    1   |    1   |    0   |     952    |        100       |
|    1   |    1   |    1   |     N/A    |        N/A       |

*Figure 15. Gyroscope Automatic LPF1 Cutoff with HPF Disabled*

For the first version of the Reefwing LSM9DS1 Library we have decided to only implement `LPF1`. If required, we will add in the additional filtering later. The code below illustrates our `setGyroBandwidth()` method.

```c++
void ReefwingLSM9DS1::setGyroBandwidth(GyroBW bandwidth) {
  uint8_t CTRL_REG1_G = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_CTRL_REG1_G);

  //  Clear the BW_G bits (0 & 1), maintain the rest
  //  0xFC = 0b11111100, BW_G = 00, bandwidth = LOW (default)
  CTRL_REG1_G &= 0xFC;

  switch(bandwidth) {
    case GyroBW::MEDIUM:    //  BW_G = 01
      CTRL_REG1_G |= 0x01;
      break;
    case GyroBW::HIGH:      //  BW_G = 10
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
```

The accelerometer bandwidth setting is similar, but `CTRL_REG6_XL` includes a bit called `BW_SCAL_ODR`, which explicitly chooses between automatic bandwidth assignment based on `ODR` and manual assignment of the anti-aliasing filter bandwidth. If automatic bandwidth is enabled for the accelerometer, the assigned bandwidth for each `ODR` option is shown in Figure 16.


|    ODR (Hz)   | Bandwidth (Hz) |
|:-------------:|:--------------:|
|      119      |       50       |
|      238      |       105      |
|      476      |       211      |
| 10, 50, & 952 |       408      |

*Figure 16. Accelerometer Auto Bandwidth Assignment by ODR*

If automatic bandwidth is disabled, then the accelerometer bandwidth needs to be manually assigned in accordance with Figure 17.

| BW_XL | Bandwidth (Hz) |
|:-----:|:--------------:|
|   00  |       408      |
|   01  |       211      |
|   10  |       105      |
|   11  |       50       |

*Figure 17. Accelerometer Manual Bandwidth Options*

If you want to set the accelerometer bandwidth manually, you need to call `enableAccelAutoBandwidth(false)` first, or the call to `setAccelBandwidth()` will be ignored. Calling `enableAccelAutoBandwidth()` prior to setting the `ODR` (gyro or accel), will ensure that the correct bandwidth is stored in the configuration variable. The code below describes the `setAccelBandwidth()` method.

``` c++
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
```

## Axis Orientation and Sign

We can reverse the sign of the gyro axes using three bits in the `ORIENT_CFG_G` register, `SignX_G`, `SignY_G`, and `SignZ_G`. The `ORIENT_CFG_G` register contents is shown in Figure 18. Enabling these bits will reverse the sign of the readings on the corresponding axis. The Reefwing Library includes a `reverseGyroSign(GyroReverseSign axis)` method, to allow setting these three bits simultaneously.

| 7 | 6 |    5    |    4    |    3    |     2    |     1    |     0    |
|:-:|:-:|:-------:|:-------:|:-------:|:--------:|:--------:|:--------:|
| 0 | 0 | SignX_G | SignY_G | SignZ_G | Orient_2 | Orient_1 | Orient_0 |

*Figure 18. LSM9DS1 Register ORIENT_CFG_G*

The bits `Orient [2:0]` in register `ORIENT_CFG_G` (`0x13`) can be used to change the IMU axis orientation, as described in Figure 19. The default is `Orient = 000`. Use the library method, `setGyroOrientation(GyroOrient orient)`, to set these bits.

| Orient[2:0] | 000 | 001 | 010 | 011 | 100 | 101 |
|:-----------:|:---:|:---:|:---:|:---:|:---:|:---:|
|    Pitch    |  X  |  X  |  Y  |  Y  |  Z  |  Z  |
|     Roll    |  Y  |  Z  |  X  |  Z  |  X  |  Y  |
|     Yaw     |  Z  |  Y  |  Z  |  X  |  Y  |  X  |

*Figure 19. IMU Axis Orientation*

## Enable Sensor Readings

The `start()` method will start things rolling when you are ready to start processing sensor readings. This method enables the output of the sensors, turns on block data update, and will auto increment the address during multibyte reads.

``` c++
void ReefwingLSM9DS1::start() {
  // Initialise the Gyro - assumes configuration is done
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
```

Block Data Update (`BDU`) avoids the reading of values (most significant and least significant parts of the data) related to different samples. When `BDU` is activated, the data registers related to each channel always contain the most recent data produced by the device. If reading a given pair (e.g., `OUT_X_H` and `OUT_X_L`, `OUT_Y_H` , and `OUT_Y_L`, `OUT_Z_H` and `OUT_Z_L`) is initiated, the refresh for that pair is blocked until both MSB and LSB parts of the data are read.

## IMU Data Available

You can check whether new sensor data is available to be read by checking the relevant status register bit.

``` c++
bool ReefwingLSM9DS1::gyroAvailable() {
	uint8_t STATUS_REG = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_STATUS_REG);
	
	return (STATUS_REG & 0x02);
}

bool ReefwingLSM9DS1::accelAvailable() {
	uint8_t STATUS_REG = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_STATUS_REG);
	
	return (STATUS_REG & 0x01);
}

bool ReefwingLSM9DS1::magAvailable() {
	uint8_t STATUS_REG_M = readRegister(LSM9DS1M_ADDRESS, LSM9DS1M_STATUS_REG_M);
	
	return (STATUS_REG_M & 0x08);
}

bool ReefwingLSM9DS1::tempAvailable() {
	uint8_t STATUS_REG = readByte(LSM9DS1AG_ADDRESS, LSM9DS1AG_STATUS_REG);
	
	return (STATUS_REG & 0x04);
}
```

## Gyroscope and Accelerometer FIFO Registers

The LSM9DS1 has 32 slots of 16-bit data FIFO registers for each of the gyroscope's three output channels (roll, pitch and yaw), and 16-bit data FIFO registers for each of the accelerometer's three output channels (x, y and z).

The FIFO memory buffer has five different modes: 

- Bypass mode (default) - FIFO is not used and remains empty. Bypass mode is also used to reset the FIFO.
- FIFO-mode - Stops collecting data when the FIFO is full, or reaches the set threshold (THS) of samples.
- Continuous mode - If the FIFO is full, the new sample overwrites the older sample.
- Continuous-to-FIFO mode - Continuous mode until interrupt trigger, then FIFO mode.
- Bypass-to-Continuous - Bypass mode until interrupt trigger, then Continuous mode.

Each mode is selected using the `FMODE [2:0]` bits in the `FIFO_CTRL` (`0x2E`) register, Figure 20. This register also contains the FIFO threshold level setting (`FTH [4:0]`). Default value: `FTH = 0000`. The `setFIFOMode(FIFOMode mode, uint8_t threshold)` method allows you to set both the `FMODE` and `FTH` at the same time.

|    7   |    6   |    5   |   4  |   3  |   2  |   1  |   0  |
|:------:|:------:|:------:|:----:|:----:|:----:|:----:|:----:|
| FMODE2 | FMODE1 | FMODE0 | FTH4 | FTH3 | FTH2 | FTH1 | FTH0 |

*Figure 20. FIFO control register, FIFO_CTRL Register (0x2E)*

The maximum `FTH` is 31 (`0x1F`), so we cap the threshold value at this number, shift the `FMODE` number 5 bits to the left and `OR` it with `FTH`, to obtain the `FIFO_CTRL` register value.

``` c++
FIFO_CTRL = (mode << 5) | FTH
```

Programmable FIFO threshold status (`FTH`), FIFO overrun events (`OVRN`) and the number of unread samples stored (`FSS`) are available in the FIFO Status Control Register, `FIFO_SRC` (`0x2F`) register, Figure 21. We have no idea why it is labelled `FIFO_SRC` and not `FIFO_SCR`!

|  7  |   6  |   5  |   4  |   3  |   2  |   1  |   0  |
|:---:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|
| FTH | OVRN | FSS5 | FSS4 | FSS3 | FSS2 | FSS1 | FSS0 |

*Figure 21. FIFO status control register, FIFO_SRC (0x2F)*

`FSS [5:0]` in `FIFO_SRC`, contains the number of unread samples. When `FSS [5:0] = 000000`, FIFO is empty, and when `FSS [5:0] = 100000`, FIFO is full and the number of unread samples is 32.

`FIFO_SRC, FTH = 1` when the number of unread samples `FSS5:0  ≥ FTH [4:0]` in `FIFO_CTRL` . If `FIFO_CTRL FTH[4:0] = 0`, then `FIFO_SRC, FTH = 0`.

| 7 |    6    | 5 |       4      |       3       |      2      |    1    |      0      |
|:-:|:-------:|:-:|:------------:|:-------------:|:-----------:|:-------:|:-----------:|
| 0 | SLEEP_G | 0 | FIFO_TEMP_EN | DRDY_mask_bit | I2C_DISABLE | FIFO_EN | STOP_ON_FTH |

*Figure 22. Control Register 9, CTRL_REG9 (0x23)*

The FIFO feature is enabled by setting `FIFO_EN =  1` in `CTRL_REG9` (`0x23`), Figure 22. To guarantee the correct acquisition of data during the switching into and out of FIFO mode, the **first sample acquired should be discarded**. We have provided the `enableFIFO(bool bitValue)` method to turn the `FIFO_EN` bit on and off.

## Read Sensor Data

Finally! After getting all that setup out of the way, we can do what we came here for, and read some IMU data!

## Read Temperature

The temperature sensor on the LSM9DS1 will read the chip temperature, which will generally be higher than ambient. From Table 5 in the LSM9DS1 Data Sheet, we note that the sensor operating range is -40 to 85 °C and the sensitivity is 16 LSB/°C. The temperature is stored across two registers (`OUT_TEMP_H` & `OUT_TEMP_L`) encoded as a two's complemented, right justified number.

``` c++
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
```

A quirk of this sensor is that it reads 0 at 25 °C, so we need to add this as an offset to get a value in Celcius. This is an approximate value, so we have included a method `setTempOffset()`, if you want to tweak the temperature reading.

## Bias Offset Calibration

It is annoying, but you need to calibrate these sensors before taking a reading. The easiest correction is the bias offset, and the IMU should be flat and motionless during offset calibration.

The gyroscope bias offset (or zero-rate level), describes the actual output signal if there is no angular rate present. This offset in MEMS sensors is a result of stress to the sensor and therefore the zero-rate level can change after mounting the sensor onto a printed circuit board or after exposing it to mechanical stress. This value changes very little over temperature and time.

The accelerometer bias offset, describes the deviation of an actual output signal from the ideal output signal if no acceleration is present. A sensor in a steady state on a horizontal surface should measure 0 g on both the X-axis and Y-axis, whereas the Z-axis will measure 1 g. Ideally, the output is in the middle of the dynamic range of the sensor (content of `OUT` registers '`0x00`'). A deviation from the ideal value in this case is called zero-g or bias offset.

The `calibrateGyro()` and `calibrateAccel()` methods will calculate and load the bias offsets. With the sensor stationary and facing up, call these methods and they will average 32 samples using `FIFO_TH` mode.

Similarly, for the magnetometer, the bias offset describes the deviation of an actual output signal from the ideal output if no magnetic field is present. The `calibrateMag()` method, takes 128 samples and calculates the maximum and minimum readings when the sensor is not moving. These maximum and minimum values are averaged for each axis (x, y, and z) and then applied as the bias offset for the magnetometer.

These bias values will be different for each sensor and could change if the board was subject to mechanical stress (e.g., a drone crash). We have provided an example sketch to show how the calibrate functions are used.

From the LSM9DS1 Data Sheet, Table 3, typical bias offsets are:

- Gyroscope (G_TyOff): ± 30 dps
- Accelerometer (LA_TyOff): ± 90 mg
- Magnetometer (M_TyOff): ± 1 gauss

This highlights the need for calibration. An uncorrected gyro offset of 30 degrees per second is material!

## Gyroscope and Accelerometer Self Test

The LSM9DS1 includes a self test function that is not particularly well documented - actually it is undocumented, until now! To start the Gyro self-test, control register 10, `CTRL_REG10` (`0x24`), bit `ST_G` needs to be set to `1`. For the accelerometer, control register 10, bit `ST_XL` is set (Figure 23).

| 7 | 6 | 5 | 4 | 3 |   2  | 1 |   0   |
|:-:|:-:|:-:|:-:|:-:|:----:|:-:|:-----:|
| 0 | 0 | 0 | 0 | 0 | ST_G | 0 | ST_XL |

*Figure 23. Control Register 10, CTRL_REG10 (0x24)*

The Reefwing library looks after this in the `selfTest()` method. It works with the bias offset methods to calculate the average of the IMU at-rest readings and then loads these resulting offsets into the accelerometer and gyroscope bias registers.

The Self Test approach that we have used is based on the ST MEMs Standard C Drivers example, [lsm9ds1_self_test.c](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm9ds1_STdC/examples/lsm9ds1_self_test.c). In summary:

- Disable self-test (`ST`).
- Calibrate the gyro and accelerometer, and store the offset bias (`gyro_noST` and `accel_noST`).
- Enable self-test.
- Calibrate the gyro and accelerometer again, and store the offset bias (`gyro_ST` and `accel_ST`).
- Calculate the difference (e.g., `gyro_ST.x - gyro_noST.x`) and scale. This is the result that is returned.

The results of the `selfTest()` function are loaded into a data structure, `SelfTestResults`, which is defined as:

``` c++
struct SelfTestResults {
  float gyrodx;
  float gyrody;
  float gyrodz;
  float accdx;
  float accdy;
  float accdz;
};
```

A sensor will PASS the self test if these results are within the expected range:

- Gyroscope x, y and z axis expected range: 20–250 dps.
- Accelerometer x, y, and z axis expected range: 60–1700 mg.

## Magnetometer Self Test

As the magnetometer in the LSM9DS1 is equivalent to the LIS3MDL, we can use the self-test procedure documented in the LIS3MDL Data Sheet (See data sheet folder in this library). This is very similar to the gyroscope and accelerometer self-test procedure.

The magnetometer PASS/FAIL self-test limits are detailed in Figure 24.

| Axis | ST Min (gauss) | ST Max (gauss) |
|:----:|:--------------:|:--------------:|
|   X  |       1.0      |       3.0      |
|   Y  |       1.0      |       3.0      |
|   Z  |       0.1      |       1.0      |

*Figure 24. Magnetometer Self-test limits (FS = 12 gauss) [Derived from the LIS3MDL Data Sheet, Table 17]*

Our interpretation of the magnetometer self-test method is:

``` c++
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

  //  Calculate difference and scale to _mRes = ± 12 gauss
  results.magdx = (mag_ST.x - mag_noST.x) * SENSITIVITY_MAGNETOMETER_12;
  results.magdy = (mag_ST.y - mag_noST.y) * SENSITIVITY_MAGNETOMETER_12;
  results.magdz = (mag_ST.z - mag_noST.z) * SENSITIVITY_MAGNETOMETER_12;

  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x1C);  //  Disable Self Test
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x03);  //  Power Down mode

  return results;
}
```

## Read Gyroscope

The angular rate values (x, y and z) are stored as a 16-bit word in two's complement, across two 8-bit registers (e.g., `OUT_X_L_G` and `OUT_X_H_G`). Once we have reconstructed the 16-bit word, we don't need to explicitly convert from 2's complement to a int_16t, since the cast does it automatically.

``` c++
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
```

The `readAccel()` and `readMag()` methods work exactly the same but with the appropriate registers for those sensors. These three methods return floats which have been scaled based on the full scale resolution which has been selected. You can also read the raw register contents, which have been converted from 2's complement encoding to 16-bit signed integers. These methods are called `readGyroRaw()`, `readAccelRaw()` and `readMagRaw()`.

## How to Use the Reefwing LSM9DS1 Library

At its simplest, a sketch will include the following:

``` c++
#include <ReefwingLSM9DS1.h>

ReefwingLSM9DS1 imu;

void setup() {
  imu.begin(); // Initialise the LSM9DS1 IMU
  
  //  Start Serial and wait for a connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 
    //  Start processing sensor data
    imu.start();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }
}

void loop() {
  //  Update the SensorData struct with the latest readings
  imu.updateSensorData(); 
  
  //  Print Gyro data to Serial Monitor
  Serial.print("Gyro X: ");
  Serial.print(imu.data.gx);
  Serial.print("\tGyro Y: ");
  Serial.print(imu.data.gy);
  Serial.print("\tGyro Z: ");
  Serial.print(imu.data.gz);
  Serial.println(" DPS");
}
```

The SensorData struct is updated with new sensor readings (if available), every time `update()` is called. The SensorData struct is shown below.

``` c++
struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};
```

If you want accurate readings, make sure that you calibrate the sensors as shown in the example sketches.

## Reefwing LSM9DS1 Library Examples

The following example sketches are provided with the library:

- `readTemperature.ino` - Read the LSM9DS1 Gyro chip temperature.
- `simpleGyroscope.ino` - Display gyroscope sensor data, with default configuration on either the Arduino Serial Monitor or Serial Plotter.
- `simpleAccelerometer.ino` - Display accelerometer sensor data, with default configuration on either the Arduino Serial Monitor or Serial Plotter.
- `simpleMagnetometer.ino` - Display magnetometer sensor data, with default configuration on either the Arduino Serial Monitor or Serial Plotter.
- `simpleIMU.ino` - Shows a different technique to display and update data for all three sensors.
- `selfTest.ino` - Demonstrate the IMU self test capability (gyroscope, accelerometer and magnetometer).
- `imuConfiguration.ino` - If you don't want to use the default IMU configuration, this sketch shows how to change the sensor settings.

These examples should give you a good idea of the libraries capabilities. It is also worth having a look at the header files or `keywords.txt` file to get an idea of the public methods and data types available.

Suggestions, corrections and comments on this Library are welcome!