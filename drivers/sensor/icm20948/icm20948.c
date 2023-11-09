/*
 * Copyright (c) 2023, Lazaro O'Farrill
 * email: lazaroofarrill@gmail.com
 *
 */

#include "zephyr/sys/printk.h"
#include <math.h>
#define DT_DRV_COMPAT invensense_icm20948

#include "zephyr/kernel.h"
#include "zephyr/sys/util.h"
#include <stdint.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include "icm20948.h"

//
// Created by Lazaro O'Farrill on 09/03/2023.
//

LOG_MODULE_REGISTER(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

static int icm20948_bank_select(const struct device *dev, uint8_t bank)
{
	if (bank > 3) {
		return -EINVAL;
	}

	const struct icm20948_config *cfg = dev->config;

	const uint8_t reg_bank_mask = 0x30;
	int ret = i2c_reg_update_byte_dt(&cfg->i2c, REG_BANK_SEL, reg_bank_mask, bank << 4);
	if (ret) {
		LOG_ERR("Error switching banks");
		return ret;
	}

	return 0;
}

static inline void icm20948_temp_c(int32_t in, int32_t *out_c, uint32_t *out_uc)
{
	/* Offset by 21 degrees Celsius */
	int64_t in100 = (in * 100) + (ROOM_TEMP_OFFSET_DEG * TEMP_SENSITIVITY_X100);

	/* Whole celsius */
	*out_c = in100 / TEMP_SENSITIVITY_X100;

	/* Micro celsius */
	*out_uc = ((in100 - (*out_c) * TEMP_SENSITIVITY_X100) * INT64_C(1000000)) /
		  TEMP_SENSITIVITY_X100;
}

static void icm20948_convert_temp(struct sensor_value *val, int16_t raw_val)
{
	icm20948_temp_c((int32_t)raw_val, &val->val1, &val->val2);
}

static inline void icm20948_accel_mss(int64_t sensitivity, int32_t in, int32_t *out, int32_t *u_out)
{
	/* Convert to micrometers/s^2 */
	int64_t in_ms = in * SENSOR_G;

	/* meters/s^2 whole values */
	*out = in_ms / (sensitivity * 1000000LL);

	/* micrometers/s^2 */
	*u_out = (in_ms - (*out * sensitivity * 1000000LL)) / sensitivity;
};

static void icm20948_convert_accel(const struct icm20948_config *cfg, int32_t raw_accel_value,
				   struct sensor_value *output_value)
{
	int64_t sensitivity = 0; /* value equivalent for 1g */

	switch (cfg->accel_fs) {
	case ACCEL_FS_SEL_2G:
		sensitivity = 16384;
		break;
	case ACCEL_FS_SEL_4G:
		sensitivity = 8192;
		break;
	case ACCEL_FS_SEL_8G:
		sensitivity = 4096;
		break;
	case ACCEL_FS_SEL_16G:
		sensitivity = 2048;
		break;
	}

	icm20948_accel_mss(sensitivity, raw_accel_value, &output_value->val1, &output_value->val2);
}

static int icm20948_convert_gyro(struct sensor_value *val, int16_t raw_val, uint8_t gyro_fs)
{
	int64_t gyro_sensitivity_10x = 0; /* value equivalent for 10x gyro reading deg/s */

	switch (gyro_fs) {
	case GYRO_FS_250:
		gyro_sensitivity_10x = 1310;
		break;
	case GYRO_FS_500:
		gyro_sensitivity_10x = 655;
		break;
	case GYRO_FS_1000:
		gyro_sensitivity_10x = 328;
		break;
	case GYRO_FS_2000:
		gyro_sensitivity_10x = 164;
		break;
	default:
		return -EINVAL;
	}

	int64_t in10_rads = (int64_t)raw_val * SENSOR_PI * 10LL;

	/* Whole rad/s */
	val->val1 = (int32_t)(in10_rads / (gyro_sensitivity_10x * 180LL * 1000000LL));

	/* microrad/s */
	val->val2 = (int32_t)((in10_rads - (val->val1 * gyro_sensitivity_10x * 180LL * 1000000LL)) /
			      (gyro_sensitivity_10x * 180LL));

	return 0;
}

static inline void icm20948_mag_gauss(int32_t in, int32_t *out, int32_t *u_out)
{

	int32_t k_magnetic_flux_sensitivity_x100 = 15;

	int64_t in100_gauss = in * 1000000LL; // 1 GAUSS equals 100 uT

	*out = (int32_t)(in100_gauss / (k_magnetic_flux_sensitivity_x100 * 1000000LL));

	*u_out = (int32_t)(in100_gauss - (*out * k_magnetic_flux_sensitivity_x100 * 1000000LL)) /
		 k_magnetic_flux_sensitivity_x100;
};

static int icm20948_convert_mag(struct sensor_value *val, int16_t raw_val)
{
	icm20948_mag_gauss((int32_t)raw_val, &val->val1, &val->val2);

	return 0;
}

static int icm20948_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct icm20948_data *dev_data = dev->data;
	const struct icm20948_config *cfg = dev->config;

	switch (chan) {
	case SENSOR_CHAN_DIE_TEMP:
		icm20948_convert_temp(val, dev_data->temp);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20948_convert_accel(cfg, dev_data->accel_x, val);
		icm20948_convert_accel(cfg, dev_data->accel_y, val + 1);
		icm20948_convert_accel(cfg, dev_data->accel_z, val + 2);
		break;
	case SENSOR_CHAN_ACCEL_X:
		icm20948_convert_accel(cfg, dev_data->accel_x, val);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm20948_convert_accel(cfg, dev_data->accel_y, val);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm20948_convert_accel(dev->config, dev_data->accel_z, val);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20948_convert_gyro(val, dev_data->gyro_x, cfg->gyro_fs);
		icm20948_convert_gyro(val + 1, dev_data->gyro_y, cfg->gyro_fs);
		icm20948_convert_gyro(val + 2, dev_data->gyro_z, cfg->gyro_fs);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm20948_convert_gyro(val, dev_data->gyro_x, cfg->gyro_fs);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm20948_convert_gyro(val, dev_data->gyro_y, cfg->gyro_fs);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm20948_convert_gyro(val, dev_data->gyro_z, cfg->gyro_fs);
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		icm20948_convert_mag(val, dev_data->mag_x);
		icm20948_convert_mag(val + 1, dev_data->mag_y);
		icm20948_convert_mag(val + 2, dev_data->mag_z);
		break;
	case SENSOR_CHAN_MAGN_X:
		icm20948_convert_mag(val, dev_data->mag_x);
		break;
	case SENSOR_CHAN_MAGN_Y:
		icm20948_convert_mag(val, dev_data->mag_y);
		break;
	case SENSOR_CHAN_MAGN_Z:
		icm20948_convert_mag(val, dev_data->mag_z);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static bool icm20948_ready_to_read(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;
	uint8_t ready_to_read = 0;

	int ret = i2c_reg_read_byte_dt(&cfg->i2c, INT_STATUS_1, &ready_to_read);
	if (ret) {
		LOG_ERR("data not ready to read.\n");
		return false;
	}

	return ready_to_read;
}

static int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct icm20948_data *drv_data = dev->data;
	const struct icm20948_config *cfg = dev->config;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL); // TODO support single channel fetch

	uint8_t read_buff[SENS_READ_BUFF_LEN];

	icm20948_bank_select(dev, 0);

	if (!icm20948_ready_to_read(dev)) {
		return -EBUSY;
	}

	int err = i2c_burst_read_dt(&cfg->i2c, ACCEL_XOUT_H, read_buff, SENS_READ_BUFF_LEN);
	if (err) {
		LOG_ERR("Error reading acc and gyro values");
		return err;
	}

	drv_data->accel_x = (int16_t)(read_buff[0] << 8 | read_buff[1]);
	drv_data->accel_y = (int16_t)(read_buff[2] << 8 | read_buff[3]);
	drv_data->accel_z = (int16_t)(read_buff[4] << 8 | read_buff[5]);
	drv_data->gyro_x = (int16_t)(read_buff[6] << 8 | read_buff[7]);
	drv_data->gyro_y = (int16_t)(read_buff[8] << 8 | read_buff[9]);
	drv_data->gyro_z = (int16_t)(read_buff[10] << 8 | read_buff[11]);
	drv_data->temp = (int16_t)(read_buff[12] << 8 | read_buff[13]);

	// check for mag overflow in AK09916_ST2
	//	if (!((BIT(3) & read_buff[20]) >> 3)) {
	drv_data->mag_x = (int16_t)(read_buff[15] << 8 | read_buff[14]);
	drv_data->mag_y = (int16_t)(read_buff[17] << 8 | read_buff[16]);
	drv_data->mag_z = (int16_t)(read_buff[19] << 8 | read_buff[18]);
	//	}

	printk("fetched values: ");
	for (int i = 0; i < SENS_READ_BUFF_LEN; i++) {
		printk("%6d", read_buff[i]);
	}
	printk("\n\n");

	return 0;
}

static const struct sensor_driver_api icm20948_driver_api = {.sample_fetch = icm20948_sample_fetch,
							     .channel_get = icm20948_channel_get};

static int icm20948_wake_up(const struct device *dev)
{
	icm20948_bank_select(dev, 0);

	const struct icm20948_config *cfg = dev->config;

	int err = i2c_reg_update_byte_dt(&cfg->i2c, PWR_MGMT_1, BIT(6), 0x00);
	if (err) {
		LOG_ERR("Error waking up device.");
		return err;
	}

	return 0;
}

static int icm20948_reset(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;

	icm20948_bank_select(dev, 0);

	int err = i2c_reg_update_byte_dt(&cfg->i2c, PWR_MGMT_1, BIT(7), 0xFF);
	if (err) {
		LOG_ERR("Error resetting device.");
		return err;
	}

	k_sleep(K_MSEC(120)); // wait for sensor to ramp up after resetting

	return 0;
}

static int icm20948_gyro_config(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;

	int err = icm20948_bank_select(dev, 2);
	if (err) {
		return err;
	}

	err = i2c_reg_update_byte_dt(&cfg->i2c, GYRO_CONFIG_1, GENMASK(5, 0),
				     GYRO_DLPFCFG_0 | cfg->gyro_fs << 1 | GYRO_F_ENABLE);
	if (err) {
		return err;
	}

	return 0;
}

static int icm20948_accel_config(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;

	int err = icm20948_bank_select(dev, 2);
	if (err) {
		return err;
	}

	err = i2c_reg_update_byte_dt(&cfg->i2c, ACCEL_CONFIG, GENMASK(5, 0),
				     ACCEL_DLPFCFG_0 | (cfg->accel_fs << 1) | ACCEL_FCHOICE_ENABLE);
	if (err) {
		return err;
	}

	return 0;
}

/*@brief Control I2C_MST_BYPASS
 */
static int icm20948_bypass(const struct device *dev, bool enable)
{
	const struct icm20948_config *cfg = dev->config;
	icm20948_bank_select(dev, 0);

	int err = i2c_reg_update_byte_dt(&cfg->i2c, USER_CTRL, BIT(5), (!enable) << 5);
	if (err) {
		LOG_ERR("Error enabling I2C_MASTER.\n");
		return err;
	}

	int ret = i2c_reg_update_byte_dt(&cfg->i2c, INT_PIN_CFG, BIT(1), enable << 1);
	if (ret) {
		return ret;
	}

	return 0;
}

typedef enum {
	AK09916_MODE_PWR_DWN = 0x00,
	AK09916_MODE_SINGLE = 0x01,
	AK09916_MODE_CONT_1 = 0x02,
	AK09916_MODE_CONT_2 = 0x04,
	AK09916_MODE_CONT_3 = 0x06,
	AK09916_MODE_CONT_4 = 0x08,
	AK09916_MODE_SELF_TEST = 0x10,
} ak09916_mode;

static int icm20948_mag_config_clk(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;
	int err = icm20948_bank_select(dev, 3);
	if (err) {
		return err;
	}

	// Set I2C master clock frequency
	err = i2c_reg_update_byte_dt(&cfg->i2c, I2C_MST_CTRL, GENMASK(3, 0), 0x07);
	if (err) {
		LOG_ERR("Couldn't set i2c master clock frequency");
		return err;
	}

	return 0;
}

static int icm20948_mag_config_mode(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;
	int err = icm20948_bank_select(dev, 3);
	if (err) {
		return err;
	}

	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_ADDR, AK09916_I2C_ADDR);
	if (err) {
		return err;
	}

	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_REG, AK09916_CNTL2);
	if (err) {
		return err;
	}

	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_DO, 0x08);
	if (err) {
		return err;
	}

	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_CTRL, 0);
	if (err) {
		LOG_ERR("Could not configure sensor to read mag data");
		return err;
	}

	k_sleep(K_USEC(1000));

	return 0;
}

static int icm20948_mag_config_reads(const struct device *dev)
{

	const struct icm20948_config *cfg = dev->config;

	int err = icm20948_bank_select(dev, 3);
	if (err) {
		return err;
	}

	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_ADDR, BIT(7) | AK09916_I2C_ADDR);
	if (err) {
		LOG_ERR("Couldn't set read address for AK09916");
		return err;
	}

	// configuring mag to read from hxl to st2
	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_REG, AK09916_HXL);
	if (err) {
		return err;
	}

	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_CTRL, BIT(7) | 9);
	if (err) {
		LOG_ERR("Could not configure sensor to read mag data");
		return err;
	}

	err = icm20948_bank_select(dev, 0);
	if (err) {
		return err;
	}

	return 0;
}

static int icm20948_mag_config(const struct device *dev)
{

	int err = icm20948_bypass(dev, false);
	if (err) {
		return err;
	}

	err = icm20948_mag_config_clk(dev);
	if (err) {
		LOG_ERR("Error setting I2C Master clock speed.");
		return err;
	}

	err = icm20948_mag_config_mode(dev);
	if (err) {
		LOG_ERR("Error selecting mode for magnetometer.");
		return err;
	}

	err = icm20948_mag_config_reads(dev);
	if (err) {
		LOG_ERR("Error stablishing reads in the magnetometer");
		return err;
	}

	LOG_INF("AK09916 Initialized");

	return 0;
}

static int icm20948_init(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus is not ready.");
		return -ENODEV;
	}

	icm20948_reset(dev);

	icm20948_wake_up(dev);

	icm20948_accel_config(dev);

	icm20948_gyro_config(dev);

	icm20948_mag_config(dev);

	return 0;
}

#define ICM2048_DEFINE(inst)                                                                       \
	static struct icm20948_data icm20948_data_##inst;                                          \
                                                                                                   \
	static const struct icm20948_config icm20948_config_##inst = {                             \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.accel_fs = DT_INST_ENUM_IDX(inst, accel_fs),                                      \
		.gyro_fs = DT_INST_ENUM_IDX(inst, gyro_fs)};                                       \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icm20948_init, NULL, &icm20948_data_##inst,             \
				     &icm20948_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &icm20948_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM2048_DEFINE)
