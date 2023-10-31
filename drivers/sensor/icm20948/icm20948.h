#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20948_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20948_H_

#include "zephyr/drivers/i2c.h"

struct icm20948_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;

	int16_t temp;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	int16_t magn_x;
	int16_t magn_y;
	int16_t magn_z;
};

struct icm20948_dev_config {
	struct i2c_dt_spec i2c;
	uint8_t accel_fs;
	uint16_t gyro_fs;
};

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

typedef enum {
	GYRO_FS_250 = 0 << 1,
	GYRO_FS_500 = 1 << 1,
	GYRO_FS_1000 = 2 << 1,
	GYRO_FS_2000 = 3 << 1,
} gyro_fs_sel;

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM20948_H_ */
