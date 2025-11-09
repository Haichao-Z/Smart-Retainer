/**
 * @file bno055_driver.c  
 * @brief BNO055 9-DOF IMU driver with improved calibration handling
 * 
 * Improvements in this version:
 * - Smart calibration warning (rate-limited)
 * - Better handling of IMU mode (no magnetometer warnings)
 * - Improved quaternion validation
 * - Configurable calibration requirements
 */

#include "bno055_driver.h"
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(bno055_driver, LOG_LEVEL_DBG);

/* Device handle */
static const struct device *i2c_dev = NULL;
static uint8_t bno055_addr = BNO055_ADDRESS_A;
static bool driver_initialized = false;
static bool use_radians = false;
static bool calibration_loaded = false;

/* Current operation mode */
static bno055_opmode_t current_mode = BNO055_OPERATION_MODE_NDOF;

/* Stored calibration offsets */
static bno055_offsets_t saved_offsets;
static bool offsets_valid = false;

/* I2C device tree node */
#define I2C_NODE DT_NODELABEL(i2c1)

/* Calibration save flag */
static bool auto_save_calibration = true;

/* Calibration warning state */
static struct {
    uint32_t last_warning_time;
    uint32_t warning_interval_ms;
    bool warning_enabled;
} calib_warning = {
    .last_warning_time = 0,
    .warning_interval_ms = 5000,  /* Warn at most every 5 seconds */
    .warning_enabled = true
};

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

/**
 * @brief Write multiple bytes to BNO055
 */
static int bno055_write_bytes(uint8_t reg, const uint8_t *buffer, uint8_t len)
{
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], buffer, len);
    return i2c_write(i2c_dev, buf, len + 1, bno055_addr);
}

/**
 * @brief Get calibration offsets from sensor
 */
int bno055_get_offsets(bno055_offsets_t *offsets)
{
    int ret;
    uint8_t mode;
    
    if (!driver_initialized || !offsets) {
        return -EINVAL;
    }
    
    /* Save current mode */
    ret = bno055_read_reg(BNO055_OPR_MODE_ADDR, &mode, 1);
    if (ret) return ret;
    
    /* Switch to config mode */
    ret = bno055_write_reg(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    if (ret) return ret;
    k_msleep(25);
    
    /* Read accelerometer offsets */
    ret = bno055_read_bytes(BNO055_ACCEL_OFFSET_X_LSB_ADDR, 
                            (uint8_t*)offsets->accel_offset, 6);
    if (ret) goto restore_mode;
    
    /* Read magnetometer offsets */
    ret = bno055_read_bytes(BNO055_MAG_OFFSET_X_LSB_ADDR, 
                            (uint8_t*)offsets->mag_offset, 6);
    if (ret) goto restore_mode;
    
    /* Read gyroscope offsets */
    ret = bno055_read_bytes(BNO055_GYRO_OFFSET_X_LSB_ADDR, 
                            (uint8_t*)offsets->gyro_offset, 6);
    if (ret) goto restore_mode;
    
    /* Read radii */
    ret = bno055_read_bytes(BNO055_ACCEL_RADIUS_LSB_ADDR, 
                            (uint8_t*)&offsets->accel_radius, 2);
    if (ret) goto restore_mode;
    
    ret = bno055_read_bytes(BNO055_MAG_RADIUS_LSB_ADDR, 
                            (uint8_t*)&offsets->mag_radius, 2);
    
restore_mode:
    /* Restore previous mode */
    bno055_write_reg(BNO055_OPR_MODE_ADDR, mode);
    k_msleep(25);
    
    if (ret == 0) {
        LOG_INF("Retrieved calibration offsets successfully");
    }
    
    return ret;
}

/**
 * @brief Set calibration offsets to sensor
 */
int bno055_set_offsets(const bno055_offsets_t *offsets)
{
    int ret;
    uint8_t mode;
    
    if (!driver_initialized || !offsets) {
        return -EINVAL;
    }
    
    /* Save current mode */
    ret = bno055_read_reg(BNO055_OPR_MODE_ADDR, &mode, 1);
    if (ret) return ret;
    
    /* Switch to config mode */
    ret = bno055_write_reg(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    if (ret) return ret;
    k_msleep(25);
    
    /* Write accelerometer offsets */
    ret = bno055_write_bytes(BNO055_ACCEL_OFFSET_X_LSB_ADDR, 
                             (uint8_t*)offsets->accel_offset, 6);
    if (ret) goto restore_mode;
    
    /* Write magnetometer offsets */
    ret = bno055_write_bytes(BNO055_MAG_OFFSET_X_LSB_ADDR, 
                             (uint8_t*)offsets->mag_offset, 6);
    if (ret) goto restore_mode;
    
    /* Write gyroscope offsets */
    ret = bno055_write_bytes(BNO055_GYRO_OFFSET_X_LSB_ADDR, 
                             (uint8_t*)offsets->gyro_offset, 6);
    if (ret) goto restore_mode;
    
    /* Write radii */
    ret = bno055_write_bytes(BNO055_ACCEL_RADIUS_LSB_ADDR, 
                             (uint8_t*)&offsets->accel_radius, 2);
    if (ret) goto restore_mode;
    
    ret = bno055_write_bytes(BNO055_MAG_RADIUS_LSB_ADDR, 
                             (uint8_t*)&offsets->mag_radius, 2);
    
restore_mode:
    /* Restore previous mode */
    bno055_write_reg(BNO055_OPR_MODE_ADDR, mode);
    k_msleep(25);
    
    if (ret == 0) {
        LOG_INF("✅ Applied calibration offsets successfully");
        calibration_loaded = true;
    }
    
    return ret;
}

/**
 * @brief Wait for full calibration (configurable based on mode)
 */
static int wait_for_full_calibration(void)
{
    bno055_calibration_t calib;
    int ret;
    int timeout_count = 0;
    const int max_timeout = 600; /* 60 seconds */
    
    /* Determine which sensors need calibration based on mode */
    bool need_mag = (current_mode == BNO055_OPERATION_MODE_NDOF ||
                     current_mode == BNO055_OPERATION_MODE_NDOF_FMC_OFF ||
                     current_mode == BNO055_OPERATION_MODE_COMPASS ||
                     current_mode == BNO055_OPERATION_MODE_M4G);
    
    LOG_INF("╔════════════════════════════════════════╗");
    LOG_INF("║     CALIBRATION REQUIRED               ║");
    LOG_INF("╠════════════════════════════════════════╣");
    LOG_INF("║  Mode: %s", need_mag ? "NDOF (All sensors)" : "IMU (No magnetometer)");
    LOG_INF("║  Instructions:                         ║");
    LOG_INF("║   1. Gyro: Keep device STILL (3s)     ║");
    LOG_INF("║   2. Accel: 6 orientations (±X,±Y,±Z) ║");
    if (need_mag) {
        LOG_INF("║   3. Mag: Figure-8 motion (far from metal) ║");
    }
    LOG_INF("╚════════════════════════════════════════╝");
    
    while (timeout_count < max_timeout) {
        ret = bno055_get_calibration(&calib);
        if (ret) {
            LOG_ERR("Failed to get calibration: %d", ret);
            return ret;
        }
        
        /* Log progress every 2 seconds */
        if (timeout_count % 20 == 0) {
            if (need_mag) {
                LOG_INF("Calibration: Sys=%d/3 Gyro=%d/3 Accel=%d/3 Mag=%d/3",
                        calib.sys, calib.gyro, calib.accel, calib.mag);
            } else {
                LOG_INF("Calibration: Sys=%d/3 Gyro=%d/3 Accel=%d/3 (Mag not used)",
                        calib.sys, calib.gyro, calib.accel);
            }
            
            /* Provide specific guidance */
            if (calib.gyro < 3) {
                LOG_INF("  → Gyro: Keep device completely still");
            }
            if (calib.accel < 3) {
                LOG_INF("  → Accel: Place device in different orientations");
            }
            if (need_mag && calib.mag < 3) {
                LOG_INF("  → Mag: Move in figure-8 pattern away from metal");
            }
        }
        
        /* Check for full calibration based on mode */
        bool fully_calibrated;
        if (need_mag) {
            /* NDOF mode: all sensors must be calibrated */
            fully_calibrated = (calib.sys >= 3 && calib.gyro >= 3 && 
                               calib.accel >= 3 && calib.mag >= 3);
        } else {
            /* IMU mode: only gyro and accel need calibration */
            fully_calibrated = (calib.sys >= 3 && calib.gyro >= 3 && calib.accel >= 3);
        }
        
        if (fully_calibrated) {
            LOG_INF("✅ Full calibration achieved!");
            
            /* Save calibration if enabled */
            if (auto_save_calibration) {
                ret = bno055_get_offsets(&saved_offsets);
                if (ret == 0) {
                    offsets_valid = true;
                    LOG_INF("💾 Calibration offsets saved to memory");
                }
            }
            
            return 0;
        }
        
        /* Allow for partial calibration if taking too long */
        if (timeout_count > 300) {
            bool acceptable;
            if (need_mag) {
                acceptable = (calib.sys >= 2 && calib.gyro >= 2 && 
                             calib.accel >= 2 && calib.mag >= 2);
            } else {
                acceptable = (calib.sys >= 2 && calib.gyro >= 2 && calib.accel >= 2);
            }
            
            if (acceptable) {
                LOG_WRN("⚠️  Timeout approaching, accepting partial calibration");
                LOG_WRN("⚠️  Accuracy may be reduced!");
                return 0;
            }
        }
        
        k_msleep(100);
        timeout_count++;
    }
    
    LOG_ERR("❌ Calibration timeout!");
    return -ETIMEDOUT;
}

int bno055_init(const bno055_config_t *config)
{
    int ret;
    uint8_t chip_id;
    uint8_t sys_status = 0, sys_error = 0;

    LOG_INF("Initializing BNO055 driver (Enhanced version)");

    /* Get I2C device */
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    bno055_addr = config->address;
    use_radians = config->units_in_radians;
    current_mode = config->mode;

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

    LOG_INF("✅ BNO055 detected (Chip ID: 0x%02X)", chip_id);

    /* Reset to ensure clean state */
    LOG_INF("Resetting BNO055...");
    ret = bno055_write_reg(BNO055_SYS_TRIGGER_ADDR, 0x20);
    if (ret) {
        LOG_ERR("Failed to reset: %d", ret);
        return ret;
    }

    /* Wait for reset to complete */
    k_msleep(750);

    /* Verify chip ID again after reset */
    ret = bno055_read_reg(BNO055_CHIP_ID_ADDR, &chip_id, 1);
    if (ret || chip_id != BNO055_ID) {
        LOG_ERR("Failed to verify chip after reset");
        return -EIO;
    }

    /* Set to config mode */
    ret = bno055_write_reg(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    if (ret) {
        LOG_ERR("Failed to enter config mode: %d", ret);
        return ret;
    }
    k_msleep(25);

    /* Configure units */
    uint8_t unit_sel = 0x00;  /* Default: Celsius, Degrees, DPS, m/s² */
    if (use_radians) {
        unit_sel |= 0x02;  /* Enable radians */
    }
    ret = bno055_write_reg(BNO055_UNIT_SEL_ADDR, unit_sel);
    if (ret) {
        LOG_WRN("Failed to set units: %d", ret);
    }

    /* Configure external crystal if requested */
    if (config->use_external_crystal) {
        ret = bno055_write_reg(BNO055_SYS_TRIGGER_ADDR, 0x80);
        if (ret) {
            LOG_WRN("Failed to enable external crystal: %d", ret);
        } else {
            LOG_INF("✅ External crystal oscillator enabled");
        }
        k_msleep(10);
    }

    /* Set power mode to normal */
    ret = bno055_write_reg(BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    if (ret) {
        LOG_WRN("Failed to set power mode: %d", ret);
    }
    k_msleep(10);

    /* Set operation mode */
    ret = bno055_set_mode(config->mode);
    if (ret) {
        LOG_ERR("Failed to set mode: %d", ret);
        return ret;
    }

    /* Wait for sensor stabilization */
    k_msleep(100);

    /* Check system status */
    ret = bno055_get_system_status(&sys_status, &sys_error);
    if (ret == 0) {
        LOG_INF("System Status: 0x%02X, Error: 0x%02X", sys_status, sys_error);
        if (sys_error != 0) {
            LOG_WRN("⚠️  System error detected: 0x%02X", sys_error);
        }
    }

    driver_initialized = true;
    LOG_INF("✅ BNO055 initialization complete");
    LOG_INF("   Mode: %s", 
            config->mode == BNO055_OPERATION_MODE_IMU ? "IMU (Accel + Gyro)" :
            config->mode == BNO055_OPERATION_MODE_NDOF ? "NDOF (Full Fusion)" : "Other");

    return 0;
}

int bno055_set_mode(bno055_opmode_t mode)
{
    int ret;

    if (i2c_dev == NULL || !device_is_ready(i2c_dev)) {
        return -EINVAL;
    }

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

    current_mode = mode;
    LOG_INF("BNO055 mode set to 0x%02X", mode);
    return 0;
}

int bno055_read_quaternion(bno055_quaternion_t *quat)
{
    uint8_t buffer[8];
    int ret;
    bno055_calibration_t calib;
    uint32_t now;

    if (!driver_initialized || !quat) {
        return -EINVAL;
    }

    /* Check calibration status with rate limiting */
    if (calib_warning.warning_enabled) {
        ret = bno055_get_calibration(&calib);
        now = k_uptime_get_32();
        
        if (ret == 0 && (now - calib_warning.last_warning_time >= calib_warning.warning_interval_ms)) {
            /* Determine if calibration is insufficient based on mode */
            bool need_mag = (current_mode == BNO055_OPERATION_MODE_NDOF ||
                            current_mode == BNO055_OPERATION_MODE_NDOF_FMC_OFF);
            
            bool calib_insufficient;
            if (need_mag) {
                /* NDOF mode: check all sensors */
                calib_insufficient = (calib.sys < 2 || calib.gyro < 2 || 
                                     calib.accel < 2 || calib.mag < 2);
            } else {
                /* IMU mode: only check gyro and accel */
                calib_insufficient = (calib.gyro < 2 || calib.accel < 2);
            }
            
            if (calib_insufficient) {
                if (need_mag) {
                    LOG_WRN("⚠️  Calibration insufficient: Sys=%d Gyro=%d Accel=%d Mag=%d",
                            calib.sys, calib.gyro, calib.accel, calib.mag);
                } else {
                    LOG_WRN("⚠️  Calibration insufficient: Sys=%d Gyro=%d Accel=%d (IMU mode)",
                            calib.sys, calib.gyro, calib.accel);
                }
                calib_warning.last_warning_time = now;
            }
        }
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

    const double scale = 1.0 / 16384.0;
    double qw = w * scale;
    double qx = x * scale;
    double qy = y * scale;
    double qz = z * scale;
    
    /* Normalize quaternion for better accuracy */
    double norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 0.0001) {
        quat->w = (float)(qw / norm);
        quat->x = (float)(qx / norm);
        quat->y = (float)(qy / norm);
        quat->z = (float)(qz / norm);
    } else {
        /* Invalid quaternion, return identity */
        LOG_WRN("Invalid quaternion norm: %f", norm);
        quat->w = 1.0f;
        quat->x = 0.0f;
        quat->y = 0.0f;
        quat->z = 0.0f;
        return -EIO;
    }
    
    /* Validate quaternion */
    float check_norm = sqrtf(quat->w*quat->w + quat->x*quat->x + 
                            quat->y*quat->y + quat->z*quat->z);
    if (fabsf(check_norm - 1.0f) > 0.01f) {
        LOG_DBG("Quaternion normalization issue: %.4f", check_norm);
    }

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

    /* BNO055 Euler format: heading, roll, pitch as 16-bit signed integers */
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

    /* Scale: 1 LSB = 1/16 deg/s or 1/900 rad/s depending on units */
    int16_t x = (int16_t)(buffer[0] | (buffer[1] << 8));
    int16_t y = (int16_t)(buffer[2] | (buffer[3] << 8));
    int16_t z = (int16_t)(buffer[4] | (buffer[5] << 8));

    float scale = use_radians ? (1.0f / 900.0f) : (1.0f / 16.0f);
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

    /* Check based on current mode */
    bool need_mag = (current_mode == BNO055_OPERATION_MODE_NDOF ||
                    current_mode == BNO055_OPERATION_MODE_NDOF_FMC_OFF ||
                    current_mode == BNO055_OPERATION_MODE_COMPASS ||
                    current_mode == BNO055_OPERATION_MODE_M4G);

    if (need_mag) {
        return (calib.sys == 3 && calib.gyro == 3 && 
                calib.accel == 3 && calib.mag == 3);
    } else {
        /* IMU mode: magnetometer not required */
        return (calib.gyro == 3 && calib.accel == 3);
    }
}

int bno055_reset(void)
{
    int ret;

    ret = bno055_write_reg(BNO055_SYS_TRIGGER_ADDR, 0x20);
    if (ret) {
        return ret;
    }

    k_msleep(750);
    driver_initialized = false;
    calibration_loaded = false;

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

int bno055_save_calibration_profile(void)
{
    int ret;
    
    if (!bno055_is_fully_calibrated()) {
        LOG_WRN("Cannot save calibration - sensor not fully calibrated");
        return -EINVAL;
    }
    
    ret = bno055_get_offsets(&saved_offsets);
    if (ret == 0) {
        offsets_valid = true;
        LOG_INF("💾 Calibration profile saved to memory");
    }
    
    return ret;
}

int bno055_load_calibration_profile(void)
{
    if (!offsets_valid) {
        LOG_DBG("No saved calibration profile available");
        return -ENODATA;
    }
    
    return bno055_set_offsets(&saved_offsets);
}

void bno055_enable_auto_calibration_save(bool enable)
{
    auto_save_calibration = enable;
    LOG_INF("Auto calibration save: %s", enable ? "enabled" : "disabled");
}

/**
 * @brief Enable/disable calibration warnings
 */
void bno055_set_calibration_warnings(bool enable)
{
    calib_warning.warning_enabled = enable;
    LOG_INF("Calibration warnings: %s", enable ? "enabled" : "disabled");
}

/**
 * @brief Set calibration warning interval
 */
void bno055_set_warning_interval(uint32_t interval_ms)
{
    calib_warning.warning_interval_ms = interval_ms;
    LOG_INF("Calibration warning interval set to %lu ms", interval_ms);
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

    LOG_INF("╔════════════════════════════════════════╗");
    LOG_INF("║      BNO055 Self-Test                  ║");
    LOG_INF("╚════════════════════════════════════════╝");

    /* Get system status */
    ret = bno055_get_system_status(&sys_status, &sys_error);
    if (ret) {
        LOG_ERR("Failed to get system status: %d", ret);
        return ret;
    }

    LOG_INF("System Status: 0x%02X", sys_status);
    LOG_INF("System Error: 0x%02X", sys_error);

    if (sys_error != 0) {
        LOG_WRN("⚠️  System error detected!");
        switch(sys_error) {
            case 0x01: LOG_ERR("Peripheral initialization error"); break;
            case 0x02: LOG_ERR("System initialization error"); break;
            case 0x03: LOG_ERR("Self test failed"); break;
            case 0x04: LOG_ERR("Register map value out of range"); break;
            case 0x05: LOG_ERR("Register map address out of range"); break;
            case 0x06: LOG_ERR("Register map write error"); break;
            case 0x07: LOG_ERR("Low power mode not available"); break;
            case 0x08: LOG_ERR("Accelerometer power mode not available"); break;
            case 0x09: LOG_ERR("Fusion config error"); break;
            case 0x0A: LOG_ERR("Sensor config error"); break;
            default: LOG_ERR("Unknown error"); break;
        }
    }

    /* Get calibration status */
    ret = bno055_get_calibration(&calib);
    if (ret) {
        LOG_ERR("Failed to get calibration: %d", ret);
        return ret;
    }

    bool need_mag = (current_mode == BNO055_OPERATION_MODE_NDOF ||
                    current_mode == BNO055_OPERATION_MODE_NDOF_FMC_OFF);

    LOG_INF("\n📊 Calibration Status:");
    LOG_INF("  System: %d/3 %s", calib.sys, 
            calib.sys == 3 ? "✅" : calib.sys >= 2 ? "⚠️" : "❌");
    LOG_INF("  Gyro:   %d/3 %s", calib.gyro,
            calib.gyro == 3 ? "✅" : calib.gyro >= 2 ? "⚠️" : "❌");
    LOG_INF("  Accel:  %d/3 %s", calib.accel,
            calib.accel == 3 ? "✅" : calib.accel >= 2 ? "⚠️" : "❌");
    if (need_mag) {
        LOG_INF("  Mag:    %d/3 %s", calib.mag,
                calib.mag == 3 ? "✅" : calib.mag >= 2 ? "⚠️" : "❌");
    } else {
        LOG_INF("  Mag:    --- (not used in IMU mode)");
    }

    /* Read sensor data */
    ret = bno055_read_all(&data);
    if (ret) {
        LOG_ERR("Failed to read sensor data: %d", ret);
        return ret;
    }

    LOG_INF("\n🔄 Quaternion (normalized):");
    LOG_INF("  w=%.4f, x=%.4f, y=%.4f, z=%.4f",
            (double)data.quaternion.w, (double)data.quaternion.x,
            (double)data.quaternion.y, (double)data.quaternion.z);
    
    float quat_norm = sqrtf(data.quaternion.w*data.quaternion.w + 
                           data.quaternion.x*data.quaternion.x +
                           data.quaternion.y*data.quaternion.y + 
                           data.quaternion.z*data.quaternion.z);
    LOG_INF("  Norm: %.6f (should be 1.0)", (double)quat_norm);

    LOG_INF("\n📐 Euler Angles:");
    LOG_INF("  Heading: %.2f°", (double)data.euler.heading);
    LOG_INF("  Roll:    %.2f°", (double)data.euler.roll);
    LOG_INF("  Pitch:   %.2f°", (double)data.euler.pitch);

    LOG_INF("\n⬆️  Accelerometer (m/s²):");
    LOG_INF("  X=%.2f, Y=%.2f, Z=%.2f",
            (double)data.accel.x, (double)data.accel.y, (double)data.accel.z);
    
    float accel_mag = sqrtf(data.accel.x*data.accel.x + 
                           data.accel.y*data.accel.y + 
                           data.accel.z*data.accel.z);
    LOG_INF("  Magnitude: %.2f m/s² (gravity ~9.8)", (double)accel_mag);

    LOG_INF("\n🔄 Gyroscope (rad/s or deg/s):");
    LOG_INF("  X=%.3f, Y=%.3f, Z=%.3f",
            (double)data.gyro.x, (double)data.gyro.y, (double)data.gyro.z);

    if (need_mag) {
        LOG_INF("\n🧭 Magnetometer (µT):");
        LOG_INF("  X=%.2f, Y=%.2f, Z=%.2f",
                (double)data.mag.x, (double)data.mag.y, (double)data.mag.z);
        
        float mag_mag = sqrtf(data.mag.x*data.mag.x + 
                             data.mag.y*data.mag.y + 
                             data.mag.z*data.mag.z);
        LOG_INF("  Field strength: %.2f µT", (double)mag_mag);
    }

    LOG_INF("\n╔════════════════════════════════════════╗");
    LOG_INF("║  ✅ Self-Test Complete                  ║");
    LOG_INF("╚════════════════════════════════════════╝\n");

    return 0;
}