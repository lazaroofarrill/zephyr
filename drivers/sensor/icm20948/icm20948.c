/*
 * Copyright (c) 2023, Lazaro O'Farrill
 * email: lazaroofarrill@gmail.com
 *
 */

#define DT_DRV_COMPAT invensense_icm20948

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

// Common
#define FIFO_EN_1 0x66

#define REG_BANK_SEL 0x7F

//// declarations
//static int selectBank(uint8_t bank);
//
//static int wakeUp();
//
//static int reset();
//
//static int config_accel();
//
//static int config_gyro();
//
//static int config_mag();
//
//static int self_test();
//
//// definitions
//
//int initImu(uint8_t address) {
//    int err = 0;
//
//    if (address != 0x68 && address != 0x69) {
//        return -EINVAL;
//    }
//
//    err = i2c_ping(address);
//    if (err != 0) {
//        return err;
//    }
//
//    IMU.address = address;
//
//    err = reset();
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    vTaskDelay(1000 / portTICK_RATE_MS);
//
//    err = wakeUp();
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    err = config_gyro();
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    err = config_accel();
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    err = self_test();
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    return err;
//}
//
//int imu_read_sensors() {
//    int err;
//
//    const uint8_t readings[12] = {0};
//
//    // select user bank 0
//    err = selectBank(0);
//
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    err = i2c_read_register(IMU.address, ACCEL_XOUT_H, (uint8_t *) readings, 12);
//
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    IMU.accX = (((int16_t) readings[0]) << 8) | readings[1];
//    IMU.accY = (((int16_t) readings[2]) << 8) | readings[3];
//    IMU.accZ = (((int16_t) readings[4]) << 8) | readings[5];
//    IMU.gyX = (((int16_t) readings[6]) << 8) | readings[7];
//    IMU.gyY = (((int16_t) readings[8]) << 8) | readings[9];
//    IMU.gyZ = (((int16_t) readings[10]) << 8) | readings[11];
//
//    return err;
//}
//
//int selectBank(uint8_t bank) {
//    if (bank > 3) {
//        return ESP_ERR_INVALID_ARG;
//    }
//
//    int err;
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//
//    // Move head
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, (IMU.address << 1) | I2C_MASTER_WRITE, 1);
//    i2c_master_write_byte(cmd, REG_BANK_SEL, 1);
//    i2c_master_stop(cmd);
//    cmd = i2c_commit_and_create(cmd, 10, &err);
//
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    // Read REG_BANK_SEL
//    uint8_t regBankValue = 0x00;
//
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, (IMU.address << 1) | I2C_MASTER_READ, 1);
//    i2c_master_read_byte(cmd, &regBankValue, I2C_MASTER_LAST_NACK);
//    i2c_master_stop(cmd);
//    cmd = i2c_commit_and_create(cmd, 100, &err);
//
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    // Write user bank
//    regBankValue = (regBankValue & 0xcf);
//    uint8_t newUserBankValue = (bank << 4) | regBankValue;
//
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, (IMU.address << 1) | I2C_MASTER_WRITE, 1);
//    i2c_master_write_byte(cmd, REG_BANK_SEL, 1);
//    i2c_master_write_byte(cmd, newUserBankValue, 1);
//    i2c_master_stop(cmd);
//
//    // Commit
//    err = i2c_commit(cmd, 10);
//
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    return err;
//}
//

//
//

//

//
//

//
//typedef enum {
//    GYRO_F_DISABLE, // Disable low pass filter
//    GYRO_F_ENABLE   // Enable low pass filter
//} gyro_fchoice;
//
typedef enum {
    GYRO_FS_250 = 0 << 1,
    GYRO_FS_500 = 1 << 1,
    GYRO_FS_1000 = 2 << 1,
    GYRO_FS_2000 = 3 << 1,
} gyro_fs_sel;
//
//typedef enum {
//    GYRO_DLPFCFG_0 = 0 << 3,
//    GYRO_DLPFCFG_1 = 1 << 3,
//    GYRO_DLPFCFG_2 = 2 << 3,
//    GYRO_DLPFCFG_3 = 2 << 3,
//    GYRO_DLPFCFG_4 = 4 << 3,
//    GYRO_DLPFCFG_5 = 5 << 3,
//    GYRO_DLPFCFG_6 = 6 << 3,
//    GYRO_DLPFCFG_7 = 7 << 3,
//} gyro_dlpfcfg;
//
//int config_gyro() {
//    int err;
//    uint8_t gyroConfig1Reg;
//
//    err = selectBank(2);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    err = i2c_read_register(IMU.address, GYRO_CONFIG_1, &gyroConfig1Reg, 1);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    gyroConfig1Reg = utils_bit_mask(gyroConfig1Reg, 0x76, &err);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    gyroConfig1Reg = gyroConfig1Reg | GYRO_DLPFCFG_0 | GYRO_FS_250 | GYRO_F_ENABLE;
//
//    err = i2c_write_register(IMU.address, GYRO_CONFIG_1, gyroConfig1Reg);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    logger_print_statusf("Gyro configuration", "%d", gyroConfig1Reg);
//
//    return err;
//}
//
//int config_mag() {
//    return 0;
//}
//
//typedef enum {
//    AX_ST_EN_REG_DISABLE = 0 << 4,
//    AX_ST_EN_REG_ENABLE = 1 << 4,
//} ax_st_en_reg;
//
//typedef enum {
//    AY_ST_EN_REG_DISABLE = 0 << 3,
//    AY_ST_EN_REG_ENABLE = 1 << 3,
//} ay_st_en_reg;
//
//typedef enum {
//    AZ_ST_EN_REG_DISABLE = 0 << 2,
//    AZ_ST_EN_REG_ENABLE = 1 << 2,
//} az_st_en_reg;
//
//typedef enum {
//    DEC3_CFG_1_4,
//    DEC3_CFG_8,
//    DEC3_CFG_16,
//    DEC3_CFG_32,
//} dec3_cfg;
//
//int self_test() {
//    int err;
//    uint8_t accelConfig2Register;
//
//    err = selectBank(2);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    err = i2c_read_register(IMU.address, ACCEL_CONFIG_2, &accelConfig2Register, 1);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    accelConfig2Register = utils_bit_mask(accelConfig2Register, 0x75, &err);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    // Enable self test
//    accelConfig2Register = accelConfig2Register | AX_ST_EN_REG_ENABLE | AY_ST_EN_REG_ENABLE |
//                           AZ_ST_EN_REG_ENABLE;
//    err = i2c_write_register(IMU.address, ACCEL_CONFIG_2, accelConfig2Register);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    // read with self test enabled
//    err = imu_read_sensors();
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    int16_t testX = IMU.accX, testY = IMU.accY, testZ = IMU.accZ;
//
//    accelConfig2Register = utils_bit_mask(accelConfig2Register, 0x76, &err);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    // Disable self test
//    err = i2c_write_register(IMU.address, ACCEL_CONFIG_2, accelConfig2Register);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    // Read with self test disabled
//    err = imu_read_sensors();
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    // Read self test references
//    err = selectBank(1);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    uint8_t selfTestGyroRef[3], selfTestAccelRef[3];
//    err = i2c_read_register(IMU.address, SELF_TEST_X_GYRO, selfTestGyroRef, 3);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    err = i2c_read_register(IMU.address, SELF_TEST_X_ACCEL, selfTestAccelRef, 3);
//    if (err != ESP_OK) {
//        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//        return err;
//    }
//
//    logger_print_statusf("Self Test Response X", "%d --- Expected: %d", testX - IMU.accX,
//                         selfTestAccelRef[0]);
//    logger_print_statusf("Self Test Response Y", "%d --- Expected: %d", testY - IMU.accY,
//                         selfTestAccelRef[1]);
//    logger_print_statusf("Self Test Response Z", "%d --- Expected: %d", testZ - IMU.accZ,
//                         selfTestAccelRef[2]);
//
//    return err;
//}

// New code

#define ICM20948_READ_BUF_SIZE 120

LOG_MODULE_REGISTER(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

static int icm20948_bank_select(const struct device *dev, uint8_t bank) {
    if (bank > 3) {
        return -EINVAL;
    }

    const struct icm20948_config *cfg = dev->config;

    const uint8_t reg_bank_mask = 0x30;
    int ret = i2c_reg_update_byte_dt(&cfg->i2c,
                                     REG_BANK_SEL,
                                     reg_bank_mask,
                                     bank << 4);
    if (ret) {
        LOG_ERR("Error switching banks");
        return ret;
    }

    return 0;
}

static void icm20948_convert_accel(struct sensor_value *val, int16_t raw_val,
                                   uint16_t sensitivity_shift) {
    int64_t conv_val;
    conv_val = ((int64_t) raw_val * SENSOR_G) >> sensitivity_shift;
    val->val1 = (int32_t) (conv_val / 1000000);
    val->val2 = (int32_t) (conv_val % 1000000);
}

static int icm20948_convert_gyro(struct sensor_value *val, int16_t raw_val, uint8_t gyro_fs) {
    int64_t sensitivity = 0; /* value equivalent for 10x gyro reading deg/s */

    switch (gyro_fs) {
        case GYRO_FS_250:
            sensitivity = 1310;
            break;
        case GYRO_FS_500:
            sensitivity = 655;
            break;
        case GYRO_FS_1000:
            sensitivity = 328;
            break;
        case GYRO_FS_2000:
            sensitivity = 164;
            break;
        default:
            return -EINVAL;
    }

    int64_t in10_rads = (int64_t) raw_val * SENSOR_PI * 10LL;

/* Whole rad/s */
    val->val1 = (int32_t) (in10_rads / (sensitivity * 180LL * 1000000LL));

/* microrad/s */
    val->val2 = (int32_t) ((in10_rads - (val->val1 * sensitivity * 180LL * 1000000LL)) / (sensitivity * 180LL));

    return 0;
}


static int icm20948_convert_mgn(struct sensor_value *val, int16_t raw_val) {
    const double k_magnetic_flux_density = 4912.0;
    double flux_value = raw_val * k_magnetic_flux_density / INT16_MAX;
    int res = sensor_value_from_double(val, flux_value);
    if (res) {
        return res;
    }

    return 0;
}

static int icm20948_channel_get(const struct device *dev, enum sensor_channel chan,
                                struct sensor_value *val) {
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
            icm20948_convert_gyro(val, drv_data->gyro_x, drv_data->gyro_fs);
            icm20948_convert_gyro(val + 1, drv_data->gyro_y, drv_data->gyro_fs);
            icm20948_convert_gyro(val + 2, drv_data->gyro_z, drv_data->gyro_fs);
            break;
        case SENSOR_CHAN_GYRO_X:
            icm20948_convert_gyro(val, drv_data->gyro_x, drv_data->gyro_fs);
            break;
        case SENSOR_CHAN_GYRO_Y:
            icm20948_convert_gyro(val, drv_data->gyro_y, drv_data->gyro_fs);
            break;
        case SENSOR_CHAN_GYRO_Z:
            icm20948_convert_gyro(val, drv_data->gyro_z, drv_data->gyro_fs);
            break;
        case SENSOR_CHAN_MAGN_XYZ:
            icm20948_convert_mgn(val, drv_data->magn_x);
            icm20948_convert_mgn(val + 1, drv_data->magn_y);
            icm20948_convert_mgn(val + 2, drv_data->magn_z);
            break;
        case SENSOR_CHAN_MAGN_X:
            icm20948_convert_mgn(val, drv_data->magn_x);
            break;
        case SENSOR_CHAN_MAGN_Y:
            icm20948_convert_mgn(val, drv_data->magn_y);
            break;
        case SENSOR_CHAN_MAGN_Z:
            icm20948_convert_mgn(val, drv_data->magn_z);
            break;
        default:
            return -ENOTSUP;
    }
    return 0;
}

static int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct icm20948_data *drv_data = dev->data;
    const struct icm20948_config *cfg = dev->config;

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL); //TODO support single channel fetch


    uint8_t read_buff[12];


    icm20948_bank_select(dev, 0);

    int err = i2c_burst_read_dt(&cfg->i2c, ACCEL_XOUT_H, read_buff, 12);
    if (err) {
        LOG_ERR("Error reading acc and gyro values");
        return err;
    }

    drv_data->accel_x = (int16_t) (read_buff[0] << 8 | read_buff[1]);
    drv_data->accel_y = (int16_t) (read_buff[2] << 8 | read_buff[3]);
    drv_data->accel_z = (int16_t) (read_buff[4] << 8 | read_buff[5]);
    drv_data->gyro_x = (int16_t) (read_buff[6] << 8 | read_buff[7]);
    drv_data->gyro_y = (int16_t) (read_buff[8] << 8 | read_buff[9]);
    drv_data->gyro_z = (int16_t) (read_buff[10] << 8 | read_buff[11]);

    printf("accX: %d.%d\n", read_buff[0], read_buff[1]);
    printf("accY: %d.%d\n", read_buff[2], read_buff[3]);
    printf("accZ: %d.%d\n", read_buff[4], read_buff[5]);

    return 0;
}

static const struct sensor_driver_api icm20948_driver_api = {.sample_fetch = icm20948_sample_fetch,
        .channel_get = icm20948_channel_get};

int icm20948_wake_up(const struct device *dev) {

    const struct icm20948_config *cfg = dev->config;

    int err = i2c_reg_update_byte_dt(&cfg->i2c, PWR_MGMT_1, 0xBF, 0x00);
    if (err) {
        LOG_ERR("Error waking up device.");
        return err;
    }

    return 0;
}

int icm20948_reset(const struct device *dev) {
    const struct icm20948_config *cfg = dev->config;

    int err = i2c_reg_update_byte_dt(&cfg->i2c, PWR_MGMT_1, 0x80, 0xFF);
    if (err) {
        LOG_ERR("Error resetting device.");
        return err;
    }

    k_sleep(K_USEC(100));

    return 0;
}

int icm20948_accel_config(const struct device *dev) {
    const struct icm20948_config *cfg = dev->config;
    int err;

    uint8_t accelConfigReg;

    err = i2c_reg_read_byte_dt(&cfg->i2c, ACCEL_CONFIG, &accelConfigReg);
    if (err) {
        return err;
    }


    accelConfigReg = accelConfigReg & 0xC0;

    accelConfigReg = accelConfigReg | ACCEL_DLPFCFG_0 | ACCEL_FS_SEL_2G |
                     ACCEL_FCHOICE_ENABLE; //TODO make this variable configurable in the sensor

    err = i2c_reg_write_byte_dt(&cfg->i2c, ACCEL_CONFIG, accelConfigReg);
    if (err) {
        return err;
    }

    return err;
}

static int icm20948_init(const struct device *dev) {
    const struct icm20948_config *cfg = dev->config;
    struct icm20948_data *dev_data = dev->data;

    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C bus is not ready.");
        return -ENODEV;
    }

    // read gyro scale factor
    icm20948_bank_select(dev, 2);
    uint8_t reusable_register = 0; //register to reuse
    i2c_reg_read_byte_dt(&cfg->i2c, GYRO_CONFIG_1, &reusable_register);
    dev_data->gyro_fs = (reusable_register & 0x06) >> 1;

    i2c_reg_read_byte_dt(&cfg->i2c, ACCEL_CONFIG_2, &reusable_register);
    dev_data->accel_sensitivity_shift = (reusable_register & 0x06) >> 1;

    icm20948_reset(dev);

    icm20948_wake_up(dev);

    icm20948_accel_config(dev);

    return 0;
}

#define ICM2048_DEFINE(inst)                                                                       \
    static struct icm20948_data icm20948_data_##inst;                                          \
                                                                                                   \
    static const struct icm20948_config icm20948_config##inst = {                              \
        .i2c = I2C_DT_SPEC_INST_GET(inst)};                                                \
                                                                                                   \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, icm20948_init, NULL, &icm20948_data_##inst,             \
                     &icm20948_config##inst, POST_KERNEL,                          \
                     CONFIG_SENSOR_INIT_PRIORITY, &icm20948_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM2048_DEFINE)
