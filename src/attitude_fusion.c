/**
 * @file attitude_fusion.c
 * @brief Madgwick AHRS filter implementation
 */

#include "attitude_fusion.h"
#include <zephyr/logging/log.h>
#include <math.h>
#include <float.h>

/* Define M_PI if not defined */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(attitude_fusion, LOG_LEVEL_DBG);

/* Filter parameters */
static float beta_gain = 0.1f;        /* Filter gain */
static float sample_period = 0.01f;   /* Sampling period (seconds) */

/* Current quaternion state */
static quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};  /* Identity quaternion */
static bool initialized = false;

/* Use standard math library */
static inline float inv_sqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;  /* Safety check */
    }
    return 1.0f / sqrtf(x);
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

    LOG_INF("  Setting beta_gain");
    beta_gain = beta;
    
    LOG_INF("  Setting sample_period");
    sample_period = sample_period_s;
    
    LOG_INF("  Initializing quaternion");
    /* Initialize quaternion to identity */
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    
    LOG_INF("  Setting initialized flag");
    initialized = true;
    
    LOG_INF("attitude_fusion_init: Success - beta=%.3f, T=%.3f s", 
            (double)beta_gain, (double)sample_period);
    
    return 0;
}

/**
 * @brief Madgwick AHRS update algorithm
 */
int attitude_fusion_update(const imu_data_t *imu_data, attitude_t *attitude)
{
    if (!initialized || !imu_data || !attitude) {
        return -EINVAL;
    }

    float ax = imu_data->accel_x;
    float ay = imu_data->accel_y;
    float az = imu_data->accel_z;
    float gx = imu_data->gyro_x;
    float gy = imu_data->gyro_y;
    float gz = imu_data->gyro_z;

    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    float norm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2qw, _2qx, _2qy, _2qz, _4qw, _4qx, _4qy, _8qx, _8qy, qwqw, qxqx, qyqy, qzqz;

    /* Normalize accelerometer measurement */
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.0001f) {
        /* Skip update if accelerometer data is invalid */
        return 0;
    }
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* Auxiliary variables to avoid repeated calculations */
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

    /* Gradient descent algorithm corrective step */
    s0 = _4qw * qyqy + _2qy * ax + _4qw * qxqx - _2qx * ay;
    s1 = _4qx * qzqz - _2qz * ax + 4.0f * qwqw * qx - _2qw * ay - _4qx + _8qx * qxqx + _8qx * qyqy + _4qx * az;
    s2 = 4.0f * qwqw * qy + _2qw * ax + _4qy * qzqz - _2qz * ay - _4qy + _8qy * qxqx + _8qy * qyqy + _4qy * az;
    s3 = 4.0f * qxqx * qz - _2qx * ax + 4.0f * qyqy * qz - _2qy * ay;
    
    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm < 0.0001f) {
        /* Skip gradient step if norm is too small */
        norm = 1.0f;
    } else {
        norm = 1.0f / norm;
    }
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;

    /* Compute rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-qx * gx - qy * gy - qz * gz) - beta_gain * s0;
    qDot2 = 0.5f * (qw * gx + qy * gz - qz * gy) - beta_gain * s1;
    qDot3 = 0.5f * (qw * gy - qx * gz + qz * gx) - beta_gain * s2;
    qDot4 = 0.5f * (qw * gz + qx * gy - qy * gx) - beta_gain * s3;

    /* Integrate to yield quaternion */
    qw += qDot1 * sample_period;
    qx += qDot2 * sample_period;
    qy += qDot3 * sample_period;
    qz += qDot4 * sample_period;

    /* Normalize quaternion */
    norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm < 0.0001f) {
        /* Reset to identity if quaternion becomes invalid */
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
 * @brief Convert quaternion to Euler angles
 */
void quaternion_to_euler(const quaternion_t *q, euler_angles_t *euler)
{
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;

    /* Roll (X-axis rotation) */
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);

    /* Pitch (Y-axis rotation) */
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(M_PI / 2.0f, sinp);
    } else {
        euler->pitch = asinf(sinp);
    }

    /* Yaw (Z-axis rotation) */
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

/**
 * @brief Reset attitude to initial state
 */
void attitude_fusion_reset(void)
{
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    LOG_INF("Attitude fusion reset");
}

/**
 * @brief Get current attitude
 */
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