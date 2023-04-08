![version](https://img.shields.io/github/v/tag/Reefwing-Software/Reefwing-LSM9DS1) ![license](https://img.shields.io/badge/license-MIT-green) ![release](https://img.shields.io/github/release-date/Reefwing-Software/Reefwing-LSM9DS1?color="red") ![open source](https://badgen.net/badge/open/source/blue?icon=github)

# Reefwing LSM9DS1

 An Arduino Library for the LSM9DS1 9-axis IMU, Arduino Nano 33 BLE Sense Revision 1. This library differs from the ArduinoLSM9DS1 library in the following ways:

 - 






|   7  |  6  |     5     |   4   |  3  | 2          | 1   | 0        |
|:----:|:---:|:---------:|:-----:|:---:|------------|-----|----------|
| BOOT | BDU | H_LACTIVE | PP_OD | SIM | IF_ADD_INC | BLE | SW_RESET |

Control Register 8

| 7 |  6  |  5  | 4 |    3   |     2    | 1 | 0 |
|:-:|:---:|:---:|:-:|:------:|:--------:|:-:|:-:|
| 0 | FS1 | FS0 | 0 | REBOOT | SOFT_RST | 0 | 0 |

Control Register 2 M

| **FS1** | **FS0** | **Full Scale** |
|:-------:|:-------:|:--------------:|
|    0    |    0    |    ± 4 gauss   |
|    0    |    1    |    ± 8 gauss   |
|    1    |    0    |   ± 12 gauss   |
|    1    |    1    |   ± 16 gauss   |

Magnetometer Full-scale Selection

| ODR (Hz) | Power Mode | Current (mA) |
|:--------:|:----------:|:------------:|
|   14.9   |     LOW    |      1.9     |
|   59.5   |     LOW    |      2.4     |
|    119   |     LOW    |      3.1     |
|    238   |   NORMAL   |      4.3     |
|    476   |   NORMAL   |      4.3     |
|    952   |   NORMAL   |      4.3     |



|    7    |   6   | 5 | 4 |    3    |    2    |    1    |    0    |
|:-------:|:-----:|:-:|:-:|:-------:|:-------:|:-------:|:-------:|
| LP_mode | HP_EN | 0 | 0 | HPCF3_G | HPCF2_G | HPCF1_G | HPCF0_G |

Gyro Control Register 3 - Enabling Low Power Mode


|    7   |    6   |    5   |   4   |   3   | 2 |   1   |   0   |
|:------:|:------:|:------:|:-----:|:-----:|:-:|:-----:|:-----:|
| ODR_G2 | ODR_G1 | ODR_G0 | FS_G1 | FS_G0 | 0 | BW_G1 | BW_G0 |

Angular Rate Sensor Control Register 1, CTRL_REG1_G (0x10)

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

Gyro/Accelerometer Mode & Sample Rate Options [Derived from the LSM9DS1 Data Sheet - Table 46]

|    7    |    6    |    5    |    4   |    3   |      2      |    1   |    0   |
|:-------:|:-------:|:-------:|:------:|:------:|:-----------:|:------:|:------:|
| ODR_XL2 | ODR_XL1 | ODR_XL0 | FS1_XL | FS0_XL | BW_SCAL_ODR | BW_XL1 | BW_XL0 |

Linear acceleration sensor Control Register 6

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

ODR register setting (accelerometer only mode) [LSM9DS1 Data Sheet - Table 68]

|     7     |  6  |  5  |  4  |  3  |  2  |     1    |  0 |
|:---------:|:---:|:---:|:---:|:---:|:---:|:--------:|:--:|
| TEMP_COMP | OM1 | OM0 | DO2 | DO1 | DO0 | FAST_ODR | ST |

Magnetometer Control Register 1, CTRL_REG1_M

| 7 | 6 | 5 | 4 |   3  |   2  |  1  | 0 |
|:-:|:-:|:-:|:-:|:----:|:----:|:---:|:-:|
| 0 | 0 | 0 | 0 | OMZ1 | OMZ0 | BLE | 0 |

Magnetometer Control Register 4, CTRL_REG4_M

|      7      | 6 |  5 | 4 | 3 |  2  |  1  |  0  |
|:-----------:|:-:|:--:|:-:|:-:|:---:|:---:|:---:|
| I2C_DISABLE | 0 | LP | 0 | 0 | SIM | MD1 | MD0 |

Magnetometer Control Register 3, CTRL_REG3_M

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

Magnetometer ODR Configuration (FAST_ODR Disabled)

| Performance | OM1 | OM0 | ODR (Hz) |
|:-----------:|:---:|:---:|:--------:|
|     LOW     |  0  |  0  |   1000   |
|    MEDIUM   |  0  |  1  |    560   |
|     HIGH    |  1  |  0  |    300   |
|    ULTRA    |  1  |  1  |    155   |

Magnetometer ODR Configuration (FAST_ODR Enabled)

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

Gyroscope Automatic LPF1 Cutoff with HPF Disabled

|    ODR (Hz)   | Bandwidth (Hz) |
|:-------------:|:--------------:|
|      119      |       50       |
|      238      |       105      |
|      476      |       211      |
| 10, 50, & 952 |       408      |

Accelerometer Auto Bandwidth Assignment by ODR

| BW_XL | Bandwidth (Hz) |
|:-----:|:--------------:|
|   00  |       408      |
|   01  |       211      |
|   10  |       105      |
|   11  |       50       |

Accelerometer Manual Bandwidth Options

| 7 | 6 |    5    |    4    |    3    |     2    |     1    |     0    |
|:-:|:-:|:-------:|:-------:|:-------:|:--------:|:--------:|:--------:|
| 0 | 0 | SignX_G | SignY_G | SignZ_G | Orient_2 | Orient_1 | Orient_0 |

LSM9DS1 Register ORIENT_CFG_G

| Orient[2:0] | 000 | 001 | 010 | 011 | 100 | 101 |
|:-----------:|:---:|:---:|:---:|:---:|:---:|:---:|
|    Pitch    |  X  |  X  |  Y  |  Y  |  Z  |  Z  |
|     Roll    |  Y  |  Z  |  X  |  Z  |  X  |  Y  |
|     Yaw     |  Z  |  Y  |  Z  |  X  |  Y  |  X  |

IMU Axis Orientation

|    7   |    6   |    5   |   4  |   3  |   2  |   1  |   0  |
|:------:|:------:|:------:|:----:|:----:|:----:|:----:|:----:|
| FMODE2 | FMODE1 | FMODE0 | FTH4 | FTH3 | FTH2 | FTH1 | FTH0 |

FIFO control register, FIFO_CTRL Register (0x2E)

|  7  |   6  |   5  |   4  |   3  |   2  |   1  |   0  |
|:---:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|
| FTH | OVRN | FSS5 | FSS4 | FSS3 | FSS2 | FSS1 | FSS0 |

FIFO status control register, FIFO_SRC (0x2F)

| 7 |    6    | 5 |       4      |       3       |      2      |    1    |      0      |
|:-:|:-------:|:-:|:------------:|:-------------:|:-----------:|:-------:|:-----------:|
| 0 | SLEEP_G | 0 | FIFO_TEMP_EN | DRDY_mask_bit | I2C_DISABLE | FIFO_EN | STOP_ON_FTH |

Control Register 9, CTRL_REG9 (0x23)

| 7 | 6 | 5 | 4 | 3 |   2  | 1 |   0   |
|:-:|:-:|:-:|:-:|:-:|:----:|:-:|:-----:|
| 0 | 0 | 0 | 0 | 0 | ST_G | 0 | ST_XL |

Control Register 10, CTRL_REG10 (0x24)

