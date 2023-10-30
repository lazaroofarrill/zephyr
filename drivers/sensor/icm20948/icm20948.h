#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20948_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20948_H_

#include "stdint.h"
#include "zephyr/device.h"
#include "zephyr/drivers/gpio.h"
#include "zephyr/drivers/i2c.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/kernel.h"

struct icm20948_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t accel_sensitivity_shift;

	int16_t temp;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint16_t gyro_sensitivity_x10;

	int16_t magn_x;
	int16_t magn_scale_x;
	int16_t magn_y;
	int16_t magn_scale_y;
	int16_t magn_z;
	int16_t magn_scale_z;
	uint8_t magn_st2;
};

struct icm20948_config {
	const struct i2c_dt_spec i2c;
	uint8_t gyro_sr_div;
	uint8_t gyro_dlpf;
	uint8_t gyro_fs;
	uint8_t accel_fs;
	uint8_t accel_dlpf;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM20948_H_ */
