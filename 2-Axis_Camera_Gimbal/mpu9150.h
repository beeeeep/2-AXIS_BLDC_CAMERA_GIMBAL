

#ifndef MPU9150_H_
#define MPU9150_H_

#include <util/delay.h>
#include "twi.h"
#include "ak8975c.h"
#include "imu.h"

/* slave address */
#define MPU9150_ADDRESS 0x68 // address pin low (GND), 0x68 (LSB 7bits)

/* Register Map of Gyroscope and accelerometer */
#define SELF_TEST_X 0x0D
#define SELF_TEST_Y 0x0E
#define SELF_TEST_Z 0x0F
#define SELF_TEST_A 0x10

#define MPU9150_RA_XG_OFFS_USRH 0x13 //[15:0] XG_OFFS_USR
#define MPU9150_RA_XG_OFFS_USRL 0x14
#define MPU9150_RA_YG_OFFS_USRH 0x15 //[15:0] YG_OFFS_USR
#define MPU9150_RA_YG_OFFS_USRL 0x16
#define MPU9150_RA_ZG_OFFS_USRH 0x17 //[15:0] ZG_OFFS_USR
#define MPU9150_RA_ZG_OFFS_USRL 0x18
#define MPU9150_RA_SMPLRT_DIV 0x19

#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

#define MPU9150_RA_FF_THR 0x1D
#define MPU9150_RA_FF_DUR 0x1E
#define MPU9150_RA_MOT_THR 0x1F
#define MPU9150_RA_MOT_DUR 0x20
#define MPU9150_RA_ZRMOT_THR 0x21
#define MPU9150_RA_ZRMOT_DUR 0x22

#define FIFO_EN 0x23

#define MPU9150_RA_I2C_MST_CTRL 0x24
#define MPU9150_RA_I2C_SLV0_ADDR 0x25
#define MPU9150_RA_I2C_SLV0_REG 0x26
#define MPU9150_RA_I2C_SLV0_CTRL 0x27
#define MPU9150_RA_I2C_SLV1_ADDR 0x28
#define MPU9150_RA_I2C_SLV1_REG 0x29
#define MPU9150_RA_I2C_SLV1_CTRL 0x2A
#define MPU9150_RA_I2C_SLV2_ADDR 0x2B
#define MPU9150_RA_I2C_SLV2_REG 0x2C
#define MPU9150_RA_I2C_SLV2_CTRL 0x2D
#define MPU9150_RA_I2C_SLV3_ADDR 0x2E
#define MPU9150_RA_I2C_SLV3_REG 0x2F
#define MPU9150_RA_I2C_SLV3_CTRL 0x30
#define MPU9150_RA_I2C_SLV4_ADDR 0x31
#define MPU9150_RA_I2C_SLV4_REG 0x32
#define MPU9150_RA_I2C_SLV4_DO 0x33
#define MPU9150_RA_I2C_SLV4_CTRL 0x34
#define MPU9150_RA_I2C_SLV4_DI 0x35
#define MPU9150_RA_I2C_MST_STATUS 0x36

#define MPU9150_RA_INT_PIN_CFG 0x37

#define MPU9150_RA_INT_ENABLE 0x38
#define MPU9150_RA_DMP_INT_STATUS 0x39
#define MPU9150_RA_INT_STATUS 0x3A

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define MPU9150_RA_EXT_SENS_DATA_00 0x49
#define MPU9150_RA_EXT_SENS_DATA_01 0x4A
#define MPU9150_RA_EXT_SENS_DATA_02 0x4B
#define MPU9150_RA_EXT_SENS_DATA_03 0x4C
#define MPU9150_RA_EXT_SENS_DATA_04 0x4D
#define MPU9150_RA_EXT_SENS_DATA_05 0x4E
#define MPU9150_RA_EXT_SENS_DATA_06 0x4F
#define MPU9150_RA_EXT_SENS_DATA_07 0x50
#define MPU9150_RA_EXT_SENS_DATA_08 0x51
#define MPU9150_RA_EXT_SENS_DATA_09 0x52
#define MPU9150_RA_EXT_SENS_DATA_10 0x53
#define MPU9150_RA_EXT_SENS_DATA_11 0x54
#define MPU9150_RA_EXT_SENS_DATA_12 0x55
#define MPU9150_RA_EXT_SENS_DATA_13 0x56
#define MPU9150_RA_EXT_SENS_DATA_14 0x57
#define MPU9150_RA_EXT_SENS_DATA_15 0x58
#define MPU9150_RA_EXT_SENS_DATA_16 0x59
#define MPU9150_RA_EXT_SENS_DATA_17 0x5A
#define MPU9150_RA_EXT_SENS_DATA_18 0x5B
#define MPU9150_RA_EXT_SENS_DATA_19 0x5C
#define MPU9150_RA_EXT_SENS_DATA_20 0x5D
#define MPU9150_RA_EXT_SENS_DATA_21 0x5E
#define MPU9150_RA_EXT_SENS_DATA_22 0x5F
#define MPU9150_RA_EXT_SENS_DATA_23 0x60
#define MPU9150_RA_MOT_DETECT_STATUS 0x61
#define MPU9150_RA_I2C_SLV0_DO 0x63
#define MPU9150_RA_I2C_SLV1_DO 0x64
#define MPU9150_RA_I2C_SLV2_DO 0x65
#define MPU9150_RA_I2C_SLV3_DO 0x66
#define MPU9150_RA_I2C_MST_DELAY_CTRL 0x67
#define MPU9150_RA_SIGNAL_PATH_RESET 0x68
#define MPU9150_RA_MOT_DETECT_CTRL 0x69

#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define MPU9150_RA_BANK_SEL 0x6D
#define MPU9150_RA_MEM_START_ADDR 0x6E
#define MPU9150_RA_MEM_R_W 0x6F
#define MPU9150_RA_DMP_CFG_1 0x70
#define MPU9150_RA_DMP_CFG_2 0x71
#define MPU9150_RA_FIFO_COUNTH 0x72
#define MPU9150_RA_FIFO_COUNTL 0x73
#define MPU9150_RA_FIFO_R_W 0x74
#define WHO_AM_I 0x75

#define pi 3.141592

/* configuration */

typedef enum
{
  DLPF_260_Hz = 0,
  DLPF_184_Hz = 1,
  DLPF_94_Hz = 2,
  DLPF_44_Hz = 3,
  DLPF_21_Hz = 4,
  DLPF_10_Hz = 5,
  DLPF_5_Hz = 6
} mpu9150_bandwidth_dlpf;

/* accelerometer configuration */
#define ACC_SELF_TEST 0xE0
#define ACC_SENSITIVITY 16384.0

typedef enum
{
  MR_2_g = 0,
  MR_4_g = 1,
  MR_8_g = 2,
  MR_16_g = 3
} mpu9150_acc_measurement_range;

/* gyroscope configuration */
#define GYRO_SELF_TEST 0xE0
#define GYRO_SENSITIVITY 131.0

typedef enum
{
  MR_250 = 0,
  MR_500 = 1,
  MR_1000 = 2,
  MR_2000 = 3
} mpu9150_gyro_measurement_range;

/* I2C auxiliary configuration */
#define I2C_BYPASS_EN 0x02

void mpu9150_setup(uint8_t Sensor, mpu9150_bandwidth_dlpf dlpf);

void mpu9150_acc_setup(mpu9150_acc_measurement_range mr, uint8_t Sensor);

void mpu9150_gyro_setup(mpu9150_gyro_measurement_range mr, uint8_t Sensor);

void mpu9150_compass_setup_bypass(void);

void mpu9150_commpass_setup_slave(void);

uint8_t mpu9150_read_device_id(uint8_t Sensor);

int16_t mpu9150_read_temperature(uint8_t Sensor);

uint8_t mpu9150_read_int_status(uint8_t Sensor);

xyz mpu9150_read_acc_xyz_0(void);

xyz mpu9150_read_acc_xyz_1(void);

xyz mpu9150_read_gyro_xyz_0(void);

xyz mpu9150_read_gyro_xyz_1(void);

xyz mpu9150_read_compass_xyz_slave(uint8_t Sensor);

xyz mpu9150_get_acc_angles_0(void);

xyz mpu9150_get_acc_angles_1(void);

xyz mpu9150_get_gyro_angles_0(xyz oldangle);

xyz mpu9150_get_gyro_angles_1(xyz oldangle);

xyz mpu9150_get_complement_angles_0(uint8_t filter_gain);

xyz mpu9150_get_complement_angles_1(uint8_t filter_gain);

xyz mpu9150_get_compass_heading(void);

inline float Rajan_FastArcTan(float x);

inline float Rajan_FastArcTan2(float y, float x);
#endif /* MPU9150_H_ */
