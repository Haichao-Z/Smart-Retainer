/**
 * @file attitude_fusion.c
 * @brief Madgwick AHRS adapted for Y-axis gravity
 * 
 * COORDINATE SYSTEM:
 * - X: Right
 * - Y: Up (gravity is -Y, approximately -9.8 m/s²)
 * - Z: Forward
 * 
 * This is adapted from standard Madgwick which assumes Z-up.
 * We modify the gradient descent to work with Y-up configuration.
 */

#include "attitude_fusion.h"
#include <zephyr/logging/log.h>
#include <math.h>
#include <float.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(attitude_fusion, LOG_LEVEL_DBG);

/* Filter parameters */
static float beta_gain = 0.3f;
static float sample_period = 0.01f;

/* Gyro bias calibration */
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static bool bias_calibrated = false;

/* Current quaternion state */
static quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};
static bool initialized = false;

/* Motion detection */
#define MOTION_THRESHOLD 0.15f
#define ACCEL_THRESHOLD 0.5f

/**
 * @brief Calibrate gyroscope bias
 */
int attitude_fusion_calibrate_gyro(const imu_data_t *samples, int num_samples)
{
    if (!samples || num_samples < 10) {
        LOG_ERR("Invalid calibration parameters");
        return -EINVAL;
    }

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    LOG_INF("Starting gyro calibration with %d samples...", num_samples);
    LOG_INF("Keep device STATIONARY!");

    for (int i = 0; i < num_samples; i++) {
        sum_x += samples[i].gyro_x;
        sum_y += samples[i].gyro_y;
        sum_z += samples[i].gyro_z;
    }

    gyro_bias_x = sum_x / num_samples;
    gyro_bias_y = sum_y / num_samples;
    gyro_bias_z = sum_z / num_samples;

    bias_calibrated = true;

    LOG_INF("Gyro bias calibrated:");
    LOG_INF("  X: %.4f rad/s", (double)gyro_bias_x);
    LOG_INF("  Y: %.4f rad/s", (double)gyro_bias_y);
    LOG_INF("  Z: %.4f rad/s", (double)gyro_bias_z);

    return 0;
}

/**
 * @brief Initialize attitude fusion
 */
int attitude_fusion_init(float beta, float sample_period_s)
{
    LOG_INF("attitude_fusion_init: Entry");
    LOG_INF("  beta=%d/1000, period=%d/1000 s", 
            (int)(beta * 1000), (int)(sample_period_s * 1000));
    
    if (beta <= 0.0f || sample_period_s <= 0.0f) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }

    beta_gain = beta;
    sample_period = sample_period_s;
    
    /* Initialize quaternion to identity */
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    
    initialized = true;
    
    LOG_INF("attitude_fusion_init: Success");
    LOG_INF("Coordinate system: X=Right, Y=Up(gravity=-Y), Z=Forward");
    
    return 0;
}

/**
 * @brief Detect if device is stationary
 */
static bool is_stationary(float gx, float gy, float gz, float ax, float ay, float az)
{
    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    float accel_mag = sqrtf(ax*ax + ay*ay + az*az);
    float accel_deviation = fabsf(accel_mag - 9.8f);
    
    return (gyro_mag < MOTION_THRESHOLD) && (accel_deviation < ACCEL_THRESHOLD);
}

/**
 * @brief Madgwick AHRS update adapted for Y-axis gravity
 * 
 * Modified gradient descent algorithm that expects:
 * - Gravity on -Y axis (down direction)
 * - Accelerometer reads approximately [0, -9.8, 0] when upright
 */
int attitude_fusion_update(const imu_data_t *imu_data, attitude_t *attitude)
{
    if (!initialized || !imu_data || !attitude) {
        return -EINVAL;
    }

    /* Apply gyro bias correction */
    float ax = imu_data->accel_x;
    float ay = imu_data->accel_y;
    float az = imu_data->accel_z;
    float gx = imu_data->gyro_x - (bias_calibrated ? gyro_bias_x : 0.0f);
    float gy = imu_data->gyro_y - (bias_calibrated ? gyro_bias_y : 0.0f);
    float gz = imu_data->gyro_z - (bias_calibrated ? gyro_bias_z : 0.0f);

    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    float norm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2qw, _2qx, _2qy, _2qz, _4qw, _4qx, _4qy, _8qx, _8qy, qwqw, qxqx, qyqy, qzqz;

    /* Detect if stationary */
    bool stationary = is_stationary(gx, gy, gz, ax, ay, az);
    float effective_beta = stationary ? (beta_gain * 2.0f) : beta_gain;

    /* Normalize accelerometer measurement */
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.0001f) {
        return 0;
    }
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* Auxiliary variables */
    _2qw = 2.0f * qw;
    _2qx = 2.0f * qx;
    _2qy = 2.0f * qy;
    _2qz = 2.0f * qz;
    _4qw = 4.0f * qw;
    _4qx = 4.0f * qx;
    _4qy = 4.0f * qy;
    _8qx = 8.0f * qx;
    _8qy = 8.0f * qy;
    qwqw = qw * qw;
    qxqx = qx * qx;
    qyqy = qy * qy;
    qzqz = qz * qz;

    /* Gradient descent - ADAPTED FOR Y-AXIS GRAVITY
     * 
     * Standard Madgwick assumes gravity on Z: [0, 0, -1]
     * We have gravity on -Y: [0, -1, 0]
     * 
     * Gravity vector in body frame from quaternion (Y-down version):
     * gx_body = 2*(qx*qy - qw*qz)
     * gy_body = qw*qw - qx*qx + qy*qy - qz*qz  <- Should equal -1 for upright
     * gz_body = 2*(qy*qz + qw*qx)
     * 
     * Objective function: f = [ax - gx_body, ay - gy_body, az - gz_body]
     */
    
    /* Compute gradient (derivative of objective function) */
    s0 = -_2qz * (2.0f * (qx*qy - qw*qz) - ax) + 
          _2qw * (qwqw - qxqx + qyqy - qzqz - ay) +
          _2qx * (2.0f * (qy*qz + qw*qx) - az);
    
    s1 = _2qy * (2.0f * (qx*qy - qw*qz) - ax) +
         -_2qx * (qwqw - qxqx + qyqy - qzqz - ay) +
          _2qw * (2.0f * (qy*qz + qw*qx) - az);
    
    s2 = _2qx * (2.0f * (qx*qy - qw*qz) - ax) +
          _2qy * (qwqw - qxqx + qyqy - qzqz - ay) +
          _2qz * (2.0f * (qy*qz + qw*qx) - az);
    
    s3 = -_2qw * (2.0f * (qx*qy - qw*qz) - ax) +
         -_2qz * (qwqw - qxqx + qyqy - qzqz - ay) +
          _2qy * (2.0f * (qy*qz + qw*qx) - az);
    
    /* Normalize gradient */
    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm < 0.0001f) {
        norm = 1.0f;
    } else {
        norm = 1.0f / norm;
    }
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;

    /* Compute rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-qx * gx - qy * gy - qz * gz) - effective_beta * s0;
    qDot2 = 0.5f * (qw * gx + qy * gz - qz * gy) - effective_beta * s1;
    qDot3 = 0.5f * (qw * gy - qx * gz + qz * gx) - effective_beta * s2;
    qDot4 = 0.5f * (qw * gz + qx * gy - qy * gx) - effective_beta * s3;

    /* Integrate to yield quaternion */
    qw += qDot1 * sample_period;
    qx += qDot2 * sample_period;
    qy += qDot3 * sample_period;
    qz += qDot4 * sample_period;

    /* Normalize quaternion */
    norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm < 0.0001f) {
        q.w = 1.0f;
        q.x = 0.0f;
        q.y = 0.0f;
        q.z = 0.0f;
    } else {
        norm = 1.0f / norm;
        q.w = qw * norm;
        q.x = qx * norm;
        q.y = qy * norm;
        q.z = qz * norm;
    }

    /* Store results */
    attitude->quaternion = q;
    quaternion_to_euler(&q, &attitude->euler);
    attitude->timestamp = imu_data->timestamp;

    return 0;
}

/**
 * @brief Convert quaternion to Euler angles (ZYX convention)
 * 
 * For Y-up coordinate system:
 * - Roll: Rotation around X-axis (right)
 * - Pitch: Rotation around Y-axis (up)
 * - Yaw: Rotation around Z-axis (forward)
 */
void quaternion_to_euler(const quaternion_t *q, euler_angles_t *euler)
{
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;

    /* Roll (rotation around X-axis) */
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);

    /* Pitch (rotation around Y-axis) */
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(M_PI / 2.0f, sinp);
    } else {
        euler->pitch = asinf(sinp);
    }

    /* Yaw (rotation around Z-axis) */
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

void attitude_fusion_reset(void)
{
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    LOG_INF("Attitude fusion reset");
}

int attitude_fusion_get_current(attitude_t *attitude)
{
    if (!initialized || !attitude) {
        return -EINVAL;
    }

    attitude->quaternion = q;
    quaternion_to_euler(&q, &attitude->euler);
    attitude->timestamp = k_uptime_get_32();

    return 0;
}

bool attitude_fusion_is_calibrated(void)
{
    return bias_calibrated;
}