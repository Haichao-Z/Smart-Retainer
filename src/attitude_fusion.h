/**
 * @file attitude_fusion.h
 * @brief Improved attitude estimation with gyro calibration
 */

#ifndef ATTITUDE_FUSION_H
#define ATTITUDE_FUSION_H

#include "imu_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Quaternion structure */
typedef struct {
    float w;  /* Scalar part */
    float x;  /* Vector part X */
    float y;  /* Vector part Y */
    float z;  /* Vector part Z */
} quaternion_t;

/* Euler angles structure (in radians) */
typedef struct {
    float roll;   /* Roll angle (rotation around X-axis) */
    float pitch;  /* Pitch angle (rotation around Y-axis) */
    float yaw;    /* Yaw angle (rotation around Z-axis) */
} euler_angles_t;

/* Attitude data structure */
typedef struct {
    quaternion_t quaternion;    /* Orientation as quaternion */
    euler_angles_t euler;       /* Orientation as Euler angles */
    uint32_t timestamp;         /* Timestamp in milliseconds */
} attitude_t;

/**
 * @brief Initialize attitude fusion algorithm
 * 
 * @param beta Algorithm gain (typical: 0.1 to 0.5)
 * @param sample_period_s Sampling period in seconds
 * @return 0 on success, negative errno on failure
 */
int attitude_fusion_init(float beta, float sample_period_s);

/**
 * @brief Calibrate gyroscope bias (device must be stationary)
 * 
 * @param samples Array of IMU samples
 * @param num_samples Number of samples (minimum 10, recommended 100+)
 * @return 0 on success, negative errno on failure
 */
int attitude_fusion_calibrate_gyro(const imu_data_t *samples, int num_samples);

/**
 * @brief Update attitude estimation with new IMU data
 * 
 * @param imu_data Pointer to IMU data
 * @param attitude Pointer to store calculated attitude
 * @return 0 on success, negative errno on failure
 */
int attitude_fusion_update(const imu_data_t *imu_data, attitude_t *attitude);

/**
 * @brief Reset attitude to initial state
 */
void attitude_fusion_reset(void);

/**
 * @brief Get current attitude
 * 
 * @param attitude Pointer to store current attitude
 * @return 0 on success, negative errno on failure
 */
int attitude_fusion_get_current(attitude_t *attitude);

/**
 * @brief Check if gyro bias calibration is complete
 * 
 * @return true if calibrated
 */
bool attitude_fusion_is_calibrated(void);

/**
 * @brief Convert quaternion to Euler angles
 * 
 * @param q Input quaternion
 * @param euler Output Euler angles
 */
void quaternion_to_euler(const quaternion_t *q, euler_angles_t *euler);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_FUSION_H */