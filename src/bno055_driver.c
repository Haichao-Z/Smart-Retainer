/**
 * @file bno055_driver.c
 * @brief BNO055 9-DOF IMU driver implementation
 */

#include "bno055_driver.h"
#include <zephyr/logging/log.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(bno055_driver, LOG_LEVEL_DBG);

/* Device handle */
static const struct device *i2c_dev = NULL;
static uint8_t bno055_addr = BNO055_ADDRESS_A;
static bool driver_initialized = false;
static bool use_radians = false;

/* I2C device tree node */
#define I2C_NODE DT_NODELABEL(i2c1)

/**
 * @brief Write a byte to BNO055 register
 */
static int bno055_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_write(i2c_dev, buf, 2, bno055_addr);
}

/**
 * @brief Read from BNO055 register
 */
static int bno055_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    return i2c_write_read(i2c_dev, bno055_addr, &reg, 1, data, len);
}

/**
 * @brief Read multiple bytes from BNO055
 */
static int bno055_read_bytes(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    return bno055_read_reg(reg, buffer, len);
}

int bno055_init(const bno055_config_t *config)
{
    int ret;
    uint8_t chip_id;

    LOG_INF("Initializing BNO055 driver");

    /* Get I2C device */
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    bno055_addr = config->address;
    use_radians = config->units_in_radians;

    LOG_INF("I2C device ready, address: 0x%02X", bno055_addr);

    /* Read chip ID */
    ret = bno055_read_reg(BNO055_CHIP_ID_ADDR, &chip_id, 1);
    if (ret) {
        LOG_ERR("Failed to read chip ID: %d", ret);
        return ret;
    }

    if (chip_id != BNO055_ID) {
        LOG_ERR("Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, BNO055_ID);
        return -EINVAL;
    }

    LOG_INF("BNO055 detected, chip ID: 0x%02X", chip_id);

    /* Reset */
    ret = bno055_write_reg(BNO055_SYS_TRIGGER_ADDR, 0x20);
    if (ret) {
        LOG_ERR("Failed to reset: %d", ret);
        return ret;
    }

    /* Wait for reset to complete */
    k_msleep(650);

    /* Check chip ID again after reset */
    ret = bno055_read_reg(BNO055_CHIP_ID_ADDR, &chip_id, 1);
    if (ret || chip_id != BNO055_ID) {
        LOG_ERR("Chip ID verification failed after reset");
        return -EIO;
    }

    /* Set to config mode */
    ret = bno055_write_reg(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    if (ret) {
        LOG_ERR("Failed to set config mode: %d", ret);
        return ret;
    }
    k_msleep(25);

    /* Set power mode to normal */
    ret = bno055_write_reg(BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    if (ret) {
        LOG_ERR("Failed to set power mode: %d", ret);
        return ret;
    }
    k_msleep(10);

    /* Use external crystal if specified */
    if (config->use_external_crystal) {
        ret = bno055_write_reg(BNO055_SYS_TRIGGER_ADDR, 0x80);
        if (ret) {
            LOG_ERR("Failed to enable external crystal: %d", ret);
            return ret;
        }
        k_msleep(10);
    }

    /* Set units: radians for angular rate, m/s² for accel */
    uint8_t unit_sel = 0x00;  /* Default: Accel m/s², Gyro deg/s, Euler deg */
    if (use_radians) {
        unit_sel |= 0x02;  /* Euler angles in radians */
    }
    unit_sel |= 0x00;  /* Gyro in deg/s (we'll convert) */
    
    ret = bno055_write_reg(BNO055_UNIT_SEL_ADDR, unit_sel);
    if (ret) {
        LOG_ERR("Failed to set units: %d", ret);
        return ret;
    }

    /* Set axis mapping to match our coordinate system */
    /* Default BNO055: X=Right, Y=Forward, Z=Up
     * We want: X=Right, Y=Up, Z=Forward
     * So we need to swap Y and Z axes */
    ret = bno055_write_reg(BNO055_AXIS_MAP_CONFIG_ADDR, 0x24); /* X=X, Y=Z, Z=Y */
    if (ret) {
        LOG_ERR("Failed to set axis mapping: %d", ret);
        return ret;
    }

    ret = bno055_write_reg(BNO055_AXIS_MAP_SIGN_ADDR, 0x00); /* All positive */
    if (ret) {
        LOG_ERR("Failed to set axis signs: %d", ret);
        return ret;
    }

    /* Set operation mode */
    ret = bno055_set_mode(config->mode);
    if (ret) {
        LOG_ERR("Failed to set operation mode: %d", ret);
        return ret;
    }

    driver_initialized = true;
    LOG_INF("BNO055 initialized successfully in mode 0x%02X", config->mode);

    return 0;
}

int bno055_set_mode(bno055_opmode_t mode)
{
    int ret;

    /* Switch to config mode first */
    ret = bno055_write_reg(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    if (ret) {
        return ret;
    }
    k_msleep(25);

    /* Set new mode */
    ret = bno055_write_reg(BNO055_OPR_MODE_ADDR, mode);
    if (ret) {
        return ret;
    }
    k_msleep(25);

    LOG_INF("BNO055 mode set to 0x%02X", mode);
    return 0;
}

int bno055_read_quaternion(bno055_quaternion_t *quat)
{
    uint8_t buffer[8];
    int ret;

    if (!driver_initialized || !quat) {
        return -EINVAL;
    }

    ret = bno055_read_bytes(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
    if (ret) {
        LOG_ERR("Failed to read quaternion: %d", ret);
        return ret;
    }

    /* BNO055 quaternion format: w, x, y, z as 16-bit signed integers
     * Scale: 1 LSB = 1/(2^14) = 1/16384 */
    int16_t w = (int16_t)(buffer[0] | (buffer[1] << 8));
    int16_t x = (int16_t)(buffer[2] | (buffer[3] << 8));
    int16_t y = (int16_t)(buffer[4] | (buffer[5] << 8));
    int16_t z = (int16_t)(buffer[6] | (buffer[7] << 8));

    const float scale = 1.0f / 16384.0f;
    quat->w = w * scale;
    quat->x = x * scale;
    quat->y = y * scale;
    quat->z = z * scale;

    return 0;
}

int bno055_read_euler(bno055_euler_t *euler)
{
    uint8_t buffer[6];
    int ret;

    if (!driver_initialized || !euler) {
        return -EINVAL;
    }

    ret = bno055_read_bytes(BNO055_EULER_H_LSB_ADDR, buffer, 6);
    if (ret) {
        LOG_ERR("Failed to read Euler: %d", ret);
        return ret;
    }

    /* BNO055 Euler format: heading, roll, pitch as 16-bit signed integers
     * Scale: 1 LSB = 1/16 degree or 1/900 radian */
    int16_t h = (int16_t)(buffer[0] | (buffer[1] << 8));
    int16_t r = (int16_t)(buffer[2] | (buffer[3] << 8));
    int16_t p = (int16_t)(buffer[4] | (buffer[5] << 8));

    if (use_radians) {
        const float scale = 1.0f / 900.0f;
        euler->heading = h * scale;
        euler->roll = r * scale;
        euler->pitch = p * scale;
    } else {
        const float scale = 1.0f / 16.0f;
        euler->heading = h * scale;
        euler->roll = r * scale;
        euler->pitch = p * scale;
    }

    return 0;
}

int bno055_read_gyro(bno055_vector_t *gyro)
{
    uint8_t buffer[6];
    int ret;

    if (!driver_initialized || !gyro) {
        return -EINVAL;
    }

    ret = bno055_read_bytes(BNO055_GYRO_DATA_X_LSB_ADDR, buffer, 6);
    if (ret) {
        return ret;
    }

    /* Scale: 1 LSB = 1/16 deg/s = 1/900 rad/s */
    int16_t x = (int16_t)(buffer[0] | (buffer[1] << 8));
    int16_t y = (int16_t)(buffer[2] | (buffer[3] << 8));
    int16_t z = (int16_t)(buffer[4] | (buffer[5] << 8));

    /* Convert to rad/s */
    const float scale = ((float)M_PI / 180.0f) / 16.0f;
    gyro->x = x * scale;
    gyro->y = y * scale;
    gyro->z = z * scale;

    return 0;
}

int bno055_read_accel(bno055_vector_t *accel)
{
    uint8_t buffer[6];
    int ret;

    if (!driver_initialized || !accel) {
        return -EINVAL;
    }

    ret = bno055_read_bytes(BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 6);
    if (ret) {
        return ret;
    }

    /* Scale: 1 LSB = 1/100 m/s² */
    int16_t x = (int16_t)(buffer[0] | (buffer[1] << 8));
    int16_t y = (int16_t)(buffer[2] | (buffer[3] << 8));
    int16_t z = (int16_t)(buffer[4] | (buffer[5] << 8));

    const float scale = 1.0f / 100.0f;
    accel->x = x * scale;
    accel->y = y * scale;
    accel->z = z * scale;

    return 0;
}

int bno055_read_mag(bno055_vector_t *mag)
{
    uint8_t buffer[6];
    int ret;

    if (!driver_initialized || !mag) {
        return -EINVAL;
    }

    ret = bno055_read_bytes(BNO055_MAG_DATA_X_LSB_ADDR, buffer, 6);
    if (ret) {
        return ret;
    }

    /* Scale: 1 LSB = 1/16 µT */
    int16_t x = (int16_t)(buffer[0] | (buffer[1] << 8));
    int16_t y = (int16_t)(buffer[2] | (buffer[3] << 8));
    int16_t z = (int16_t)(buffer[4] | (buffer[5] << 8));

    const float scale = 1.0f / 16.0f;
    mag->x = x * scale;
    mag->y = y * scale;
    mag->z = z * scale;

    return 0;
}

int bno055_read_all(bno055_data_t *data)
{
    int ret;

    if (!driver_initialized || !data) {
        return -EINVAL;
    }

    ret = bno055_read_quaternion(&data->quaternion);
    if (ret) return ret;

    ret = bno055_read_euler(&data->euler);
    if (ret) return ret;

    ret = bno055_read_gyro(&data->gyro);
    if (ret) return ret;

    ret = bno055_read_accel(&data->accel);
    if (ret) return ret;

    ret = bno055_read_mag(&data->mag);
    if (ret) return ret;

    ret = bno055_get_calibration(&data->calibration);
    if (ret) return ret;

    data->timestamp = k_uptime_get_32();

    return 0;
}

int bno055_get_calibration(bno055_calibration_t *calib)
{
    uint8_t cal_status;
    int ret;

    if (!driver_initialized || !calib) {
        return -EINVAL;
    }

    ret = bno055_read_reg(BNO055_CALIB_STAT_ADDR, &cal_status, 1);
    if (ret) {
        return ret;
    }

    calib->sys = (cal_status >> 6) & 0x03;
    calib->gyro = (cal_status >> 4) & 0x03;
    calib->accel = (cal_status >> 2) & 0x03;
    calib->mag = cal_status & 0x03;

    return 0;
}

bool bno055_is_fully_calibrated(void)
{
    bno055_calibration_t calib;
    
    if (bno055_get_calibration(&calib) != 0) {
        return false;
    }

    return (calib.sys == 3 && calib.gyro == 3 && 
            calib.accel == 3 && calib.mag == 3);
}

int bno055_reset(void)
{
    int ret;

    ret = bno055_write_reg(BNO055_SYS_TRIGGER_ADDR, 0x20);
    if (ret) {
        return ret;
    }

    k_msleep(650);
    driver_initialized = false;

    LOG_INF("BNO055 reset complete");
    return 0;
}

bool bno055_is_ready(void)
{
    return driver_initialized && (i2c_dev != NULL) && device_is_ready(i2c_dev);
}

int bno055_get_system_status(uint8_t *sys_status, uint8_t *sys_error)
{
    int ret;

    if (!driver_initialized) {
        return -EINVAL;
    }

    if (sys_status) {
        ret = bno055_read_reg(BNO055_SYS_STATUS_ADDR, sys_status, 1);
        if (ret) return ret;
    }

    if (sys_error) {
        ret = bno055_read_reg(BNO055_SYS_ERR_ADDR, sys_error, 1);
        if (ret) return ret;
    }

    return 0;
}

int bno055_self_test(void)
{
    int ret;
    uint8_t sys_status, sys_error;
    bno055_calibration_t calib;
    bno055_data_t data;

    if (!driver_initialized) {
        LOG_ERR("BNO055 not initialized");
        return -EINVAL;
    }

    LOG_INF("=== BNO055 Self-Test ===");

    /* Get system status */
    ret = bno055_get_system_status(&sys_status, &sys_error);
    if (ret) {
        LOG_ERR("Failed to get system status: %d", ret);
        return ret;
    }

    LOG_INF("System Status: 0x%02X", sys_status);
    LOG_INF("System Error: 0x%02X", sys_error);

    if (sys_error != 0) {
        LOG_WRN("System error detected!");
    }

    /* Get calibration status */
    ret = bno055_get_calibration(&calib);
    if (ret) {
        LOG_ERR("Failed to get calibration: %d", ret);
        return ret;
    }

    LOG_INF("\nCalibration Status:");
    LOG_INF("  System: %d/3", calib.sys);
    LOG_INF("  Gyro:   %d/3", calib.gyro);
    LOG_INF("  Accel:  %d/3", calib.accel);
    LOG_INF("  Mag:    %d/3", calib.mag);

    /* Read sensor data */
    ret = bno055_read_all(&data);
    if (ret) {
        LOG_ERR("Failed to read sensor data: %d", ret);
        return ret;
    }

    LOG_INF("\nQuaternion:");
    LOG_INF("  w=%.3f, x=%.3f, y=%.3f, z=%.3f",
            (double)data.quaternion.w, (double)data.quaternion.x,
            (double)data.quaternion.y, (double)data.quaternion.z);

    LOG_INF("\nEuler Angles:");
    LOG_INF("  Heading: %.2f°", (double)data.euler.heading);
    LOG_INF("  Roll:    %.2f°", (double)data.euler.roll);
    LOG_INF("  Pitch:   %.2f°", (double)data.euler.pitch);

    LOG_INF("\nAccelerometer (m/s²):");
    LOG_INF("  X=%.2f, Y=%.2f, Z=%.2f",
            (double)data.accel.x, (double)data.accel.y, (double)data.accel.z);

    LOG_INF("\nGyroscope (rad/s):");
    LOG_INF("  X=%.2f, Y=%.2f, Z=%.2f",
            (double)data.gyro.x, (double)data.gyro.y, (double)data.gyro.z);

    LOG_INF("\nMagnetometer (µT):");
    LOG_INF("  X=%.2f, Y=%.2f, Z=%.2f",
            (double)data.mag.x, (double)data.mag.y, (double)data.mag.z);

    LOG_INF("\n=== Self-Test Complete ===\n");

    return 0;
}