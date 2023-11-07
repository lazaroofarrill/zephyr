/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "ak09916.h"
#include "icm20948.h"
#include "zephyr/kernel.h"
#include "zephyr/kernel_structs.h"

LOG_MODULE_DECLARE(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

#define I2C_READ_FLAG BIT(7)

#define AK09916_I2C_ADDR 0x0C

#define AK09916_REG_ID     0x00
#define AK09916_REG_ID_VAL 0x09

#define AK09916_ST2_OVRFL_BIT BIT(3)

#define AK09916_REG_CNTL1                 0x0A
#define AK09916_REG_CNTL1_POWERDOWN_VAL   0x00
#define AK09916_REG_CNTL1_FUSE_ROM_VAL    0x0F
#define AK09916_REG_CNTL1_16BIT_100HZ_VAL 0x16
#define AK09916_SET_MODE_DELAY_MS         1

#define AK09916_REG_CNTL2           0x31
#define AK09916_REG_CNTL3           0x32
#define AK09916_REG_CNTL3_RESET_VAL 0x01
#define AK09916_RESET_DELAY_MS      1

#define AK09916_REG_ADJ_DATA_X 0x10
#define AK09916_REG_ADJ_DATA_Y 0x11
#define AK09916_REG_ADJ_DATA_Z 0x12

#define AK09916_SCALE_TO_UG 1499

#define ICM20948_REG_I2C_SLV4_CTRL_VAL 0x80
#define ICM20948_REG_READOUT_CTRL_VAL  0x27

#define ICM20948_I2C_MST_STS_SLV4_DONE BIT(6)

int ak09916_convert_magn(struct sensor_value *val, int16_t raw_val, int16_t scale, uint8_t st2)
{
	/* The sensor device returns 10^-9 Teslas after scaling.
	 * Scale adjusts for calibration data and units
	 * So sensor instance returns Gauss units
	 */

	/* If overflow happens then value is invalid */
	if ((st2 & AK09916_ST2_OVRFL_BIT) != 0) {
		LOG_INF("Magnetometer value overflow.");
		return -EOVERFLOW;
	}

	int32_t scaled_val = (int32_t)raw_val * (int32_t)scale;

	val->val1 = scaled_val / 1000000;
	val->val2 = scaled_val % 1000000;
	return 0;
}

static int ak09916_execute_rw(const struct device *dev, uint8_t reg, bool write)
{
	/* Instruct the ICM20948 to access over its external i2c bus
	 * given device register with given details
	 */
	const struct icm20948_config *cfg = dev->config;
	uint8_t mode_bit = 0x00;
	uint8_t status;
	int ret;

	if (!write) {
		mode_bit = I2C_READ_FLAG;
	}

	/* Set target i2c address */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV4_ADDR, AK09916_I2C_ADDR | mode_bit);
	if (ret < 0) {
		LOG_ERR("Failed to write i2c target slave address.");
		return ret;
	}

	/* Set target i2c register */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV4_REG, reg);
	if (ret < 0) {
		LOG_ERR("Failed to write i2c target slave register.");
		return ret;
	}

	/* Initiate transfer  */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV4_CTRL, ICM20948_REG_I2C_SLV4_CTRL_VAL);
	if (ret < 0) {
		LOG_ERR("Failed to initiate i2c slave transfer.");
		return ret;
	}

	struct k_timer response_timeout;
	k_timer_start(&response_timeout, K_MSEC(500), K_NO_WAIT);
	/* Wait for a transfer to be ready */
	do {
		ret = i2c_reg_read_byte_dt(&cfg->i2c, I2C_MST_STATUS, &status);
		if (ret < 0) {
			LOG_ERR("Waiting for slave failed.");
			return ret;
		}

		if (k_timer_remaining_get(&response_timeout) > 0) {
			LOG_ERR("timedout weating for data read");
			return -ETIMEDOUT;
		}
	} while (!(status & ICM20948_I2C_MST_STS_SLV4_DONE));

	return 0;
}

static int ak09916_read_reg(const struct device *dev, uint8_t reg, uint8_t *data)
{
	const struct icm20948_config *cfg = dev->config;
	int ret;

	/* Execute transfer */
	ret = ak09916_execute_rw(dev, reg, false);
	if (ret < 0) {
		LOG_ERR("Failed to prepare transfer.");
		return ret;
	}

	/* Read the result */
	ret = i2c_reg_read_byte_dt(&cfg->i2c, I2C_SLV4_DI, data);
	if (ret < 0) {
		LOG_ERR("Failed to read data from slave.");
		return ret;
	}

	return 0;
}

static int ak09916_write_reg(const struct device *dev, uint8_t reg, uint8_t data)
{
	const struct icm20948_config *cfg = dev->config;
	int ret;

	/* Set the data to write */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV4_DO, data);
	if (ret < 0) {
		LOG_ERR("Failed to write data to slave.");
		return ret;
	}

	/* Execute transfer */
	ret = ak09916_execute_rw(dev, reg, true);
	if (ret < 0) {
		LOG_ERR("Failed to transfer write to slave.");
		return ret;
	}

	return 0;
}

static int ak09916_set_mode(const struct device *dev, uint8_t mode)
{
	int ret;

	ret = ak09916_write_reg(dev, AK09916_REG_CNTL1, mode);
	if (ret < 0) {
		LOG_ERR("Failed to set AK09916 mode.");
		return ret;
	}

	/* Wait for mode to change */
	k_msleep(AK09916_SET_MODE_DELAY_MS);
	return 0;
}

static int16_t ak09916_calc_adj(int16_t val)
{

	/** Datasheet says the actual register value is in 16bit output max
	 *  value of 32760 that corresponds to 4912 uT flux, yielding factor
	 *  of 0.149938.
	 *
	 *  Now Zephyr unit is Gauss, and conversion is 1T = 10^4G
	 *  -> 0.1499 * 10^4 = 1499
	 *  So if we multiply with scaling with 1499 the unit is uG.
	 *
	 *  Calculation from MPU-9250 Register Map and Descriptions
	 *  adj = (((val-128)*0.5)/128)+1
	 */
	return ((AK09916_SCALE_TO_UG * (val - 128)) / 256) + AK09916_SCALE_TO_UG;
}

static int ak09916_fetch_adj(const struct device *dev)
{
	/* Read magnetometer adjustment data from the AK09916 chip */
	struct icm20948_data *drv_data = dev->data;
	uint8_t buf;
	int ret;

	/* Change to FUSE access mode to access adjustment registers */
	ret = ak09916_set_mode(dev, AK09916_REG_CNTL1_FUSE_ROM_VAL);
	if (ret < 0) {
		LOG_ERR("Failed to set chip in fuse access mode.");
		return ret;
	}

	ret = ak09916_read_reg(dev, AK09916_REG_ADJ_DATA_X, &buf);
	if (ret < 0) {
		LOG_ERR("Failed to read adjustment data.");
		return ret;
	}
	drv_data->magn_scale_x = ak09916_calc_adj(buf);

	ret = ak09916_read_reg(dev, AK09916_REG_ADJ_DATA_Y, &buf);
	if (ret < 0) {
		LOG_ERR("Failed to read adjustment data.");
		return ret;
	}
	drv_data->magn_scale_y = ak09916_calc_adj(buf);

	ret = ak09916_read_reg(dev, AK09916_REG_ADJ_DATA_Z, &buf);
	if (ret < 0) {
		LOG_ERR("Failed to read adjustment data.");
		return ret;
	}
	drv_data->magn_scale_z = ak09916_calc_adj(buf);

	/* Change back to the powerdown mode */
	ret = ak09916_set_mode(dev, AK09916_REG_CNTL1_POWERDOWN_VAL);
	if (ret < 0) {
		LOG_ERR("Failed to set chip in power down mode.");
		return ret;
	}

	LOG_DBG("Adjustment values %d %d %d", drv_data->magn_scale_x, drv_data->magn_scale_y,
		drv_data->magn_scale_z);

	return 0;
}

static int ak09916_reset(const struct device *dev)
{
	int ret;

	/* Reset the chip -> reset all settings. */
	ret = ak09916_write_reg(dev, AK09916_REG_CNTL3, AK09916_REG_CNTL3_RESET_VAL);
	if (ret < 0) {
		LOG_ERR("Failed to reset AK09916.");
		return ret;
	}

	/* Wait for reset */
	k_msleep(AK09916_RESET_DELAY_MS);

	return 0;
}

static int ak09916_init_master(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;
	int ret;

	/* Instruct ICM20948 to use its external I2C bus as master */
	ret = i2c_reg_update_byte_dt(&cfg->i2c, USER_CTRL, BIT(5), 0xFF);
	if (ret < 0) {
		LOG_ERR("Failed to set ICM20948 master i2c mode.");
		return ret;
	}

	/* Set ICM20948 I2C bus as 400kHz and issue interrupt at data ready. */
	ret = i2c_reg_update_byte_dt(&cfg->i2c, I2C_MST_CTRL, GENMASK(3, 0), 0x07);
	if (ret < 0) {
		LOG_ERR("Failed to set ICM20948 master i2c speed.");
		return ret;
	}

	return 0;
}

static int ak09916_init_readout(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;
	int ret;

	/* Set target i2c address */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_ADDR, AK09916_I2C_ADDR | I2C_READ_FLAG);
	if (ret < 0) {
		LOG_ERR("Failed to set AK09916 slave address.");
		return ret;
	}

	/* Set target as data registers */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_REG, AK09916_HXL);
	if (ret < 0) {
		LOG_ERR("Failed to set AK09916 register address.");
		return ret;
	}

	/* Initiate readout at sample rate */
	ret = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_CTRL, ICM20948_REG_READOUT_CTRL_VAL);
	if (ret < 0) {
		LOG_ERR("Failed to init AK09916 value readout.");
		return ret;
	}

	return 0;
}

int ak09916_init(const struct device *dev)
{
	uint8_t buf;
	int ret;

	ret = ak09916_init_master(dev);
	if (ret < 0) {
		LOG_ERR("Initializing ICM20948 master mode failed.");
		return ret;
	}

	ret = ak09916_reset(dev);
	if (ret < 0) {
		LOG_ERR("Resetting AK09916 failed.");
		return ret;
	}

	/* First check that the chip says hello */
	ret = ak09916_read_reg(dev, AK09916_REG_ID, &buf);
	if (ret < 0) {
		LOG_ERR("Failed to read AK09916 chip id.");
		return ret;
	}

	if (buf != AK09916_REG_ID_VAL) {
		LOG_ERR("Invalid AK09916 chip id (0x%X).", buf);
		return -ENOTSUP;
	}

	/* Fetch calibration data */
	ret = ak09916_fetch_adj(dev);
	if (ret < 0) {
		LOG_ERR("Calibrating AK09916 failed.");
		return ret;
	}

	/* Set AK sample rate and resolution */
	ret = ak09916_set_mode(dev, AK09916_REG_CNTL1_16BIT_100HZ_VAL);
	if (ret < 0) {
		LOG_ERR("Failed set sample rate for AK09916.");
		return ret;
	}

	/* Init constant readouts at sample rate */
	ret = ak09916_init_readout(dev);
	if (ret < 0) {
		LOG_ERR("Initializing AK09916 readout failed.");
		return ret;
	}

	LOG_INF("ICM20948 Initialized.");

	return 0;
}
