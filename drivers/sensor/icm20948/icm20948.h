#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20948_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20948_H_

#include "zephyr/drivers/i2c.h"
#include <stdint.h>

// Registers
// Bank_0
#define WHO_AM_I             0x00
#define USER_CTRL            0x03
#define LP_CONFIG            0x05
#define PWR_MGMT_1           0x06
#define PWR_MGMT_2           0x07
#define INT_PIN_CFG          0x0F
#define INT_ENABLE           0x10
#define INT_ENABLE_1         0x11
#define INT_ENABLE_2         0x12
#define INT_ENABLE_3         0x13
#define I2C_MST_STATUS       0x17
#define INT_STATUS           0x19
#define INT_STATUS_1         0x1A
#define INT_STATUS_2         0x1B
#define INT_STATUS_3         0x1C
#define DELAY_TIMEH          0x28
#define DELAY_TIMEL          0x29
#define ACCEL_XOUT_H         0x2D
#define ACCEL_XOUT_L         0x2E
#define ACCEL_YOUT_H         0x2F
#define ACCEL_YOUT_L         0x30
#define ACCEL_ZOUT_H         0x31
#define ACCEL_ZOUT_L         0x32
#define GYRO_XOUT_H          0x33
#define GYRO_XOUT_L          0x34
#define GYRO_YOUT_H          0x35
#define GYRO_YOUT_L          0x36
#define GYRO_ZOUT_H          0x37
#define GYRO_ZOUT_L          0x38
#define TEMP_OUT_H           0x39
#define TEMP_OUT_L           0x3A
#define EXT_SLV_SENS_DATA_00 0x3B
#define EXT_SLV_SENS_DATA_01 0x3C
#define EXT_SLV_SENS_DATA_02 0x3D
#define EXT_SLV_SENS_DATA_03 0x3E
#define EXT_SLV_SENS_DATA_04 0x3F
#define EXT_SLV_SENS_DATA_05 0x40
#define EXT_SLV_SENS_DATA_06 0x41
#define EXT_SLV_SENS_DATA_07 0x42
#define EXT_SLV_SENS_DATA_08 0x43
#define EXT_SLV_SENS_DATA_09 0x44
#define EXT_SLV_SENS_DATA_10 0x45
#define EXT_SLV_SENS_DATA_11 0x46
#define EXT_SLV_SENS_DATA_12 0x47
#define EXT_SLV_SENS_DATA_13 0x48
#define EXT_SLV_SENS_DATA_14 0x49
#define EXT_SLV_SENS_DATA_15 0x4A
#define EXT_SLV_SENS_DATA_16 0x4B
#define EXT_SLV_SENS_DATA_17 0x4C
#define EXT_SLV_SENS_DATA_18 0x4D
#define EXT_SLV_SENS_DATA_19 0x4E
#define EXT_SLV_SENS_DATA_20 0x4F
#define EXT_SLV_SENS_DATA_21 0x50
#define EXT_SLV_SENS_DATA_22 0x51
#define EXT_SLV_SENS_DATA_23 0x52

// User Bank 1
#define SELF_TEST_X_GYRO  0x02
#define SELF_TEST_Y_GYRO  0x03
#define SELF_TEST_Z_GYRO  0x04
#define SELF_TEST_X_ACCEL 0x0E
#define SELF_TEST_Y_ACCEL 0x0F
#define SELF_TEST_Z_ACCEL 0x10

// User Bank 2
#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1   0x01
#define GYRO_CONFIG_2   0x02
#define ACCEL_CONFIG    0x14
#define ACCEL_CONFIG_2  0x15

// User bank 3
#define I2C_MST_ODR_CONFIG 0x00
#define I2C_MST_CTRL       0x01
#define I2C_MST_DELAY_CTRL 0x02
#define I2C_SLV0_ADDR      0x03
#define I2C_SLV0_REG       0x04
#define I2C_SLV0_CTRL      0x05
#define I2C_SLV0_DO        0x06

#define I2C_SLV4_ADDR 0x13
#define I2C_SLV4_REG  0x14
#define I2C_SLV4_CTRL 0x15
#define I2C_SLV4_DO   0x16
#define I2C_SLV4_DI   0x17
// Common
#define FIFO_EN_1     0x66

#define REG_BANK_SEL 0x7F

// AK09916
#define AK09916_I2C_ADDR 0x0C

#define AK09916_WIA   0x01
#define AK09916_ST1   0x10
#define AK09916_HXL   0x11
#define AK09916_HXH   0x12
#define AK09916_HYL   0x13
#define AK09916_HYH   0x14
#define AK09916_HZL   0x15
#define AK09916_HZH   0x16
#define AK09916_ST2   0x18
#define AK09916_CNTL2 0x31
#define AK09916_CNTL3 0x32
#define AK09916_TS1   0x33
#define AK09916_TS2   0x34

#define AK09916_DEVICE_ID 0x09

#define ROOM_TEMP_OFFSET_DEG  21
#define TEMP_SENSITIVITY_X100 33387
#define SENS_READ_BUFF_LEN    23

struct icm20948_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;

	int16_t temp;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
	int16_t magn_scale_x;
	int16_t magn_scale_y;
	int16_t magn_scale_z;
};

struct icm20948_config {
	struct i2c_dt_spec i2c;
	uint8_t accel_fs;
	uint8_t gyro_fs;
};

// TODO migrate to Kconfig
typedef enum {
	ACCEL_DLPFCFG_0 = 0 << 3,
	ACCEL_DLPFCFG_1 = 1 << 3,
	ACCEL_DLPFCFG_2 = 2 << 3,
	ACCEL_DLPFCFG_3 = 3 << 3,
	ACCEL_DLPFCFG_4 = 4 << 3,
	ACCEL_DLPFCFG_5 = 5 << 3,
	ACCEL_DLPFCFG_6 = 6 << 3,
	ACCEL_DLPFCFG_7 = 7 << 3,
} icm20948_accel_dlpfcfg;

typedef enum {
	ACCEL_FS_SEL_2G = 0,
	ACCEL_FS_SEL_4G = 1,
	ACCEL_FS_SEL_8G = 2,
	ACCEL_FS_SEL_16G = 3
} accel_fs_sel;

typedef enum {
	ACCEL_FCHOICE_DISABLE,
	ACCEL_FCHOICE_ENABLE
} accel_fchoice;

// TODO migrate to Kconfig
typedef enum {
	GYRO_FS_250 = 0,
	GYRO_FS_500 = 1,
	GYRO_FS_1000 = 2,
	GYRO_FS_2000 = 3,
} gyro_fs_sel;

// TODO migrate to Kconfig
typedef enum {
	GYRO_DLPFCFG_0 = 0 << 3,
	GYRO_DLPFCFG_1 = 1 << 3,
	GYRO_DLPFCFG_2 = 2 << 3,
	GYRO_DLPFCFG_3 = 2 << 3,
	GYRO_DLPFCFG_4 = 4 << 3,
	GYRO_DLPFCFG_5 = 5 << 3,
	GYRO_DLPFCFG_6 = 6 << 3,
	GYRO_DLPFCFG_7 = 7 << 3,
} gyro_dlpfcfg;

typedef enum {
	GYRO_F_DISABLE, // Disable low pass filter
	GYRO_F_ENABLE   // Enable low pass filter
} gyro_fchoice;

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM20948_H_ */
