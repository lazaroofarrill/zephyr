/*
 * Copyright (c) 2023, Lazaro O'Farrill
 * email: lazaroofarrill@gmail.com
 *
 */

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
#define I2C_MST_CTRL       0x01
#define I2C_MST_DELAY_CTRL 0x02
#define I2C_SLV0_ADDR      0x03
#define I2C_SLV0_REG       0x04
#define I2C_SLV0_CTRL      0x05
#define I2C_SLV0_DO        0x06

// Common
#define FIFO_EN_1 0x66

#define REG_BANK_SEL 0x7F

// AK09916
#define AK09916_HXL   0x11
#define AK09916_HXH   0x12
#define AK09916_HYL   0x13
#define AK09916_HYH   0x14
#define AK09916_HZL   0x15
#define AK09916_HZH   0x16
#define AK09916_ST2   0x18
#define AK09916_CNTL2 0x31

#define ROOM_TEMP_OFFSET_DEG  21
#define TEMP_SENSITIVITY_X100 33387
#define SENS_READ_BUFF_LEN    21

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

	/* Convert to micrometers/s^2 */
	int64_t in_ms = raw_accel_value * SENSOR_G;

	/* meters/s^2 whole values */
	output_value->val1 = in_ms / (sensitivity * 1000000LL);

	/* micrometers/s^2 */
	output_value->val2 = (in_ms - (output_value->val1 * sensitivity * 1000000LL)) / sensitivity;
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
	if (!((GENMASK(3, 3) & read_buff[20]) >> 3)) {
		drv_data->mag_x = (int16_t)(read_buff[14] << 8 | read_buff[15]);
		drv_data->mag_x = (int16_t)(read_buff[16] << 8 | read_buff[17]);
		drv_data->mag_x = (int16_t)(read_buff[18] << 8 | read_buff[19]);
	}

	// read accel config
	uint8_t reg;
	err = i2c_reg_read_byte_dt(&cfg->i2c, ACCEL_CONFIG, &reg);
	if (err) {
		return err;
	}

	return 0;
}

static const struct sensor_driver_api icm20948_driver_api = {.sample_fetch = icm20948_sample_fetch,
							     .channel_get = icm20948_channel_get};

static int icm20948_wake_up(const struct device *dev)
{
	icm20948_bank_select(dev, 0);

	const struct icm20948_config *cfg = dev->config;

	int err = i2c_reg_update_byte_dt(&cfg->i2c, PWR_MGMT_1, GENMASK(6, 6), 0x00);
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

	int err = i2c_reg_update_byte_dt(&cfg->i2c, PWR_MGMT_1, GENMASK(7, 7), 0xFF);
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
	int err;

	err = icm20948_bank_select(dev, 2);
	if (err) {
		return err;
	}

	uint8_t gyro_config_reg_1;

	err = i2c_reg_read_byte_dt(&cfg->i2c, GYRO_CONFIG_1, &gyro_config_reg_1);
	if (err) {
		return err;
	}

	gyro_config_reg_1 = gyro_config_reg_1 & GENMASK(7, 6);
	if (err) {
		return err;
	}

	gyro_config_reg_1 = gyro_config_reg_1 | GYRO_DLPFCFG_0 | GYRO_FS_250 | GYRO_F_ENABLE;

	err = i2c_reg_write_byte_dt(&cfg->i2c, GYRO_CONFIG_1, gyro_config_reg_1);
	if (err) {
		return err;
	}

	LOG_INF("Gyro Config: %d\n", gyro_config_reg_1);

	return err;
}

static int icm20948_accel_config(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;
	int err;

	uint8_t accel_config_reg;

	err = i2c_reg_read_byte_dt(&cfg->i2c, ACCEL_CONFIG, &accel_config_reg);
	if (err) {
		return err;
	}

	accel_config_reg = accel_config_reg & GENMASK(7, 6);

	accel_config_reg =
		accel_config_reg | ACCEL_DLPFCFG_0 | (cfg->accel_fs << 1) |
		ACCEL_FCHOICE_ENABLE; // TODO make this variable configurable in the sensor

	err = i2c_reg_write_byte_dt(&cfg->i2c, ACCEL_CONFIG, accel_config_reg);
	if (err) {
		return err;
	}

	return err;
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

static int icm20948_mag_config(const struct device *dev)
{
	const struct icm20948_config *cfg = dev->config;

	icm20948_bank_select(dev, 3);

	// set I2C_MST_EN
	int err = i2c_reg_update_byte_dt(&cfg->i2c, USER_CTRL, GENMASK(5, 5), 0xFF);
	if (err) {
		LOG_ERR("Error enabling I2C_MASTER.\n");
		return err;
	}

	// set I2C_MST_CLK to 400kHz
	err = i2c_reg_update_byte_dt(&cfg->i2c, I2C_MST_CTRL, GENMASK(3, 0), 7);
	if (err) {
		LOG_ERR("Setting I2C_MST clock frequency.\n");
		return err;
	}

	// set delay to 0
	err = i2c_reg_update_byte_dt(&cfg->i2c, I2C_MST_DELAY_CTRL, 0x01, 0x01);
	if (err) {
		return err;
	}

	// set slave address to magnetometer address in write mode
	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_ADDR, 0xC0);
	if (err) {
		return err;
	}

	// set write head to mag CNTL2
	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_REG, AK09916_CNTL2);
	if (err) {
		return err;
	}

	// set magnetometer to continous mode 4
	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_DO, AK09916_MODE_CONT_4);
	if (err) {
		return err;
	}

	// enable slave 0 in icm20948
	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_CTRL, GENMASK(7, 6) | 0x7);
	if (err) {
		return err;
	}

	// set slave address to magnetometer address in read mode
	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_ADDR, GENMASK(7, 7) | 0xC0);
	if (err) {
		return err;
	}

	// set where to start reading from
	err = i2c_reg_write_byte_dt(&cfg->i2c, I2C_SLV0_REG, AK09916_HXL);
	if (err) {
		return err;
	}

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
