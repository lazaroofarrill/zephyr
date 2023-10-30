/*
 * Copyright (c) 2023, Lazaro O'Farrill
 * email: lazaroofarrill@gmail.com
 *
 */

#include "zephyr/drivers/sensor.h"
#include <errno.h>
#include <stdint.h>
#define DT_DRV_COMPAT invensense_icm20948

#include "zephyr/logging/log.h"
#include "zephyr/devicetree.h"
#include "zephyr/sys/byteorder.h"

#include "icm20948.h"

#define ICM20948_READ_BUF_SIZE 120

LOG_MODULE_REGISTER(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

static void icm20948_convert_accel(struct sensor_value *val, int16_t raw_val,
				   uint16_t sensitivity_shift)
{
	int64_t conv_val;
	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

static void icm20948_convert_gyro(struct sensor_value *val, int16_t raw_val,
				  uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) / (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

static int icm20948_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct icm20948_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20948_convert_accel(val, drv_data->accel_x, drv_data->accel_sensitivity_shift);
		icm20948_convert_accel(val + 1, drv_data->accel_y,
				       drv_data->accel_sensitivity_shift);
		icm20948_convert_accel(val + 2, drv_data->accel_z,
				       drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_X:
		icm20948_convert_accel(val, drv_data->accel_x, drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm20948_convert_accel(val, drv_data->accel_y, drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm20948_convert_accel(val, drv_data->accel_z, drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20948_convert_gyro(val, drv_data->gyro_x, drv_data->gyro_sensitivity_x10);
		icm20948_convert_gyro(val + 1, drv_data->gyro_y, drv_data->gyro_sensitivity_x10);
		icm20948_convert_gyro(val + 2, drv_data->gyro_z, drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm20948_convert_gyro(val, drv_data->gyro_x, drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm20948_convert_gyro(val, drv_data->gyro_y, drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm20948_convert_gyro(val, drv_data->gyro_z, drv_data->gyro_sensitivity_x10);
		break;
	default:
		return -ENOTSUP;
	}
}

static int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct icm20948_data *drv_data = dev->data;
	const struct icm20948_config *cfg = dev->config;

	int16_t buf[ICM20948_READ_BUF_SIZE];

	LOG_ERR("TODO");
}
static const struct sensor_driver_api icm20948_driver_api = {.sample_fetch = icm20948_};

static int icm20948_init(const struct device *dev)
{
	struct icm20948_data *drv_data = dev->data;
}
