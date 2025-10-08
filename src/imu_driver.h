/**
 * @file imu_driver.h
 * @brief LSM6DS0 IMU driver interface for Smart Retainer
 * 
 * This module handles communication with LSM6DS0 6-axis IMU
 * via I2C and provides raw sensor data
 */

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IMU data structure */
typedef struct {
    float accel_x;  /* Acceleration X-axis (m/s²) */
    float accel_y;  /* Acceleration Y-axis (m/s²) */
    float accel_z;  /* Acceleration Z-axis (m/s²) */
    float gyro_x;   /* Angular velocity X-axis (rad/s) */
    float gyro_y;   /* Angular velocity Y-axis (rad/s) */
    float gyro_z;   /* Angular velocity Z-axis (rad/s) */
    uint32_t timestamp; /* Timestamp in milliseconds */
} imu_data_t;

/* IMU configuration structure */
typedef struct {
    uint16_t sample_rate_hz;    /* Sampling rate in Hz */
    uint8_t accel_range_g;      /* Accelerometer range in g */
    uint16_t gyro_range_dps;    /* Gyroscope range in degrees/sec */
} imu_config_t;

/**
 * @brief Initialize IMU driver
 * 
 * @param config Pointer to configuration structure
 * @return 0 on success, negative errno on failure
 */
int imu_driver_init(const imu_config_t *config);

/**
 * @brief Read raw IMU data
 * 
 * @param data Pointer to store the IMU data
 * @return 0 on success, negative errno on failure
 */
int imu_driver_read(imu_data_t *data);

/**
 * @brief Check if IMU is ready
 * 
 * @return true if IMU is initialized and ready
 */
bool imu_driver_is_ready(void);

/**
 * @brief Perform IMU self-test
 * 
 * @return 0 on success, negative errno on failure
 */
int imu_driver_self_test(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_DRIVER_H */