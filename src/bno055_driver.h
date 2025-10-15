/**
 * @file bno055_driver.h
 * @brief BNO055 9-DOF IMU driver interface
 * 
 * BNO055 provides built-in sensor fusion with quaternion output
 */

#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BNO055 I2C Addresses */
#define BNO055_ADDRESS_A                0x28
#define BNO055_ADDRESS_B                0x29
#define BNO055_ID                       0xA0

/* Register Map */
#define BNO055_CHIP_ID_ADDR             0x00
#define BNO055_PAGE_ID_ADDR             0x07
#define BNO055_ACCEL_DATA_X_LSB_ADDR    0x08
#define BNO055_MAG_DATA_X_LSB_ADDR      0x0E
#define BNO055_GYRO_DATA_X_LSB_ADDR     0x14
#define BNO055_EULER_H_LSB_ADDR         0x1A
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_GRAVITY_DATA_X_LSB_ADDR  0x2E
#define BNO055_TEMP_ADDR                0x34
#define BNO055_CALIB_STAT_ADDR          0x35
#define BNO055_SYS_STATUS_ADDR          0x39
#define BNO055_SYS_ERR_ADDR             0x3A
#define BNO055_UNIT_SEL_ADDR            0x3B
#define BNO055_OPR_MODE_ADDR            0x3D
#define BNO055_PWR_MODE_ADDR            0x3E
#define BNO055_SYS_TRIGGER_ADDR         0x3F
#define BNO055_AXIS_MAP_CONFIG_ADDR     0x41
#define BNO055_AXIS_MAP_SIGN_ADDR       0x42

/* Operation Modes */
typedef enum {
    BNO055_OPERATION_MODE_CONFIG        = 0x00,
    BNO055_OPERATION_MODE_ACCONLY       = 0x01,
    BNO055_OPERATION_MODE_MAGONLY       = 0x02,
    BNO055_OPERATION_MODE_GYRONLY       = 0x03,
    BNO055_OPERATION_MODE_ACCMAG        = 0x04,
    BNO055_OPERATION_MODE_ACCGYRO       = 0x05,
    BNO055_OPERATION_MODE_MAGGYRO       = 0x06,
    BNO055_OPERATION_MODE_AMG           = 0x07,
    BNO055_OPERATION_MODE_IMUPLUS       = 0x08,
    BNO055_OPERATION_MODE_COMPASS       = 0x09,
    BNO055_OPERATION_MODE_M4G           = 0x0A,
    BNO055_OPERATION_MODE_NDOF_FMC_OFF  = 0x0B,
    BNO055_OPERATION_MODE_NDOF          = 0x0C
} bno055_opmode_t;

/* Power Modes */
typedef enum {
    BNO055_POWER_MODE_NORMAL            = 0x00,
    BNO055_POWER_MODE_LOWPOWER          = 0x01,
    BNO055_POWER_MODE_SUSPEND           = 0x02
} bno055_powermode_t;

/* Quaternion structure */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} bno055_quaternion_t;

/* Euler angles structure (in degrees or radians based on config) */
typedef struct {
    float heading;  /* Yaw */
    float roll;
    float pitch;
} bno055_euler_t;

/* Vector structure */
typedef struct {
    float x;
    float y;
    float z;
} bno055_vector_t;

/* Calibration status */
typedef struct {
    uint8_t sys;    /* System calibration (0-3) */
    uint8_t gyro;   /* Gyroscope calibration (0-3) */
    uint8_t accel;  /* Accelerometer calibration (0-3) */
    uint8_t mag;    /* Magnetometer calibration (0-3) */
} bno055_calibration_t;

/* IMU data structure */
typedef struct {
    bno055_quaternion_t quaternion;
    bno055_euler_t euler;
    bno055_vector_t gyro;
    bno055_vector_t accel;
    bno055_vector_t mag;
    bno055_calibration_t calibration;
    uint32_t timestamp;
} bno055_data_t;

/* Configuration structure */
typedef struct {
    uint8_t address;                    /* I2C address */
    bno055_opmode_t mode;              /* Operation mode */
    bool use_external_crystal;          /* Use external crystal */
    bool units_in_radians;             /* Euler angles in radians */
} bno055_config_t;

/**
 * @brief Initialize BNO055 driver
 * 
 * @param config Pointer to configuration structure
 * @return 0 on success, negative errno on failure
 */
int bno055_init(const bno055_config_t *config);

/**
 * @brief Set operation mode
 * 
 * @param mode Operation mode
 * @return 0 on success, negative errno on failure
 */
int bno055_set_mode(bno055_opmode_t mode);

/**
 * @brief Read quaternion data
 * 
 * @param quat Pointer to store quaternion data
 * @return 0 on success, negative errno on failure
 */
int bno055_read_quaternion(bno055_quaternion_t *quat);

/**
 * @brief Read Euler angles
 * 
 * @param euler Pointer to store Euler angles
 * @return 0 on success, negative errno on failure
 */
int bno055_read_euler(bno055_euler_t *euler);

/**
 * @brief Read gyroscope data
 * 
 * @param gyro Pointer to store gyroscope data (rad/s or deg/s)
 * @return 0 on success, negative errno on failure
 */
int bno055_read_gyro(bno055_vector_t *gyro);

/**
 * @brief Read accelerometer data
 * 
 * @param accel Pointer to store accelerometer data (m/s²)
 * @return 0 on success, negative errno on failure
 */
int bno055_read_accel(bno055_vector_t *accel);

/**
 * @brief Read magnetometer data
 * 
 * @param mag Pointer to store magnetometer data (µT)
 * @return 0 on success, negative errno on failure
 */
int bno055_read_mag(bno055_vector_t *mag);

/**
 * @brief Read all sensor data
 * 
 * @param data Pointer to store all sensor data
 * @return 0 on success, negative errno on failure
 */
int bno055_read_all(bno055_data_t *data);

/**
 * @brief Get calibration status
 * 
 * @param calib Pointer to store calibration status
 * @return 0 on success, negative errno on failure
 */
int bno055_get_calibration(bno055_calibration_t *calib);

/**
 * @brief Check if sensor is fully calibrated
 * 
 * @return true if fully calibrated (all values = 3)
 */
bool bno055_is_fully_calibrated(void);

/**
 * @brief Perform system reset
 * 
 * @return 0 on success, negative errno on failure
 */
int bno055_reset(void);

/**
 * @brief Check if BNO055 is ready
 * 
 * @return true if initialized and ready
 */
bool bno055_is_ready(void);

/**
 * @brief Get system status
 * 
 * @param sys_status Pointer to store system status
 * @param sys_error Pointer to store system error
 * @return 0 on success, negative errno on failure
 */
int bno055_get_system_status(uint8_t *sys_status, uint8_t *sys_error);

/**
 * @brief Perform self test
 * 
 * @return 0 on success, negative errno on failure
 */
int bno055_self_test(void);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_DRIVER_H */