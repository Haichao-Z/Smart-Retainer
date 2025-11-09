/**
 * @file orientation_offset.c
 * @brief Enhanced orientation offset management with drift prevention
 * 
 * Improvements:
 * - Double precision quaternion calculations
 * - Multiple samples for zero-point calibration
 * - Better quaternion normalization
 * - Persistent storage support
 * - Drift detection and compensation
 */

#include <math.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include "orientation_offset.h"
#include "bno055_driver.h"

LOG_MODULE_REGISTER(orientation_offset, LOG_LEVEL_DBG);

/* Zero-point offset quaternion (inverse/conjugate of calibration position) */
static quaternion_offset_t zero_offset = {1.0f, 0.0f, 0.0f, 0.0f};
static bool offset_is_set = false;

/* 外部变量声明 - 用于漂移恢复 */
extern volatile bool calibrate_zero_point;

/* Calibration statistics */
static struct {
    uint32_t samples_collected;
    double acc_w, acc_x, acc_y, acc_z;  /* Accumulators for averaging */
    float last_norm_deviation;
    uint32_t drift_corrections;
} calib_stats = {0};

/* Drift detection thresholds */
#define QUATERNION_NORM_THRESHOLD   0.005f  /* Maximum deviation from unit norm */
#define DRIFT_CORRECTION_INTERVAL   1000    /* Check every N samples */

/**
 * @brief Quaternion conjugate (for inverse rotation)
 * The conjugate of a unit quaternion represents the inverse rotation
 */
static void quaternion_conjugate(const bno055_quaternion_t *q, 
                                 quaternion_offset_t *q_conj)
{
    q_conj->w = q->w;
    q_conj->x = -q->x;
    q_conj->y = -q->y;
    q_conj->z = -q->z;
}

/**
 * @brief High-precision quaternion normalization
 * Uses double precision to minimize rounding errors
 */
static void quaternion_normalize_precise(quaternion_offset_t *q)
{
    double w = q->w, x = q->x, y = q->y, z = q->z;
    double norm = sqrt(w*w + x*x + y*y + z*z);
    
    if (norm > 0.000001) {  /* Prevent division by zero */
        q->w = (float)(w / norm);
        q->x = (float)(x / norm);
        q->y = (float)(y / norm);
        q->z = (float)(z / norm);
    } else {
        /* Reset to identity quaternion if norm is too small */
        LOG_WRN("Quaternion norm too small, resetting to identity");
        q->w = 1.0f;
        q->x = q->y = q->z = 0.0f;
    }
}

/**
 * @brief High-precision quaternion multiplication
 * q_result = q1 * q2
 * Uses double precision internally for better accuracy
 */
static void quaternion_multiply_precise(const quaternion_offset_t *q1,
                                        const bno055_quaternion_t *q2,
                                        bno055_quaternion_t *result)
{
    /* Use double precision for intermediate calculations */
    double w1 = q1->w, x1 = q1->x, y1 = q1->y, z1 = q1->z;
    double w2 = q2->w, x2 = q2->x, y2 = q2->y, z2 = q2->z;
    
    /* Quaternion multiplication formula */
    double rw = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    double rx = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    double ry = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    double rz = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    
    /* Normalize the result to prevent drift accumulation */
    double norm = sqrt(rw*rw + rx*rx + ry*ry + rz*rz);
    
    if (norm > 0.000001) {
        result->w = (float)(rw / norm);
        result->x = (float)(rx / norm);
        result->y = (float)(ry / norm);
        result->z = (float)(rz / norm);
    } else {
        LOG_ERR("Quaternion multiplication resulted in zero norm!");
        result->w = 1.0f;
        result->x = result->y = result->z = 0.0f;
    }
    
    /* Track norm deviation for drift detection */
    calib_stats.last_norm_deviation = fabsf((float)norm - 1.0f);
    if (calib_stats.last_norm_deviation > QUATERNION_NORM_THRESHOLD) {
        LOG_DBG("Norm deviation: %.6f", calib_stats.last_norm_deviation);
    }
}

/**
 * @brief Validate quaternion values
 */
static bool quaternion_is_valid(const bno055_quaternion_t *q)
{
    /* Check for NaN or Inf */
    if (!isfinite(q->w) || !isfinite(q->x) || 
        !isfinite(q->y) || !isfinite(q->z)) {
        LOG_ERR("Quaternion contains invalid values (NaN/Inf)");
        return false;
    }
    
    /* Check norm is close to 1 */
    float norm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (fabsf(norm - 1.0f) > 0.1f) {
        LOG_WRN("Quaternion norm out of range: %.4f", norm);
        return false;
    }
    
    return true;
}

void orientation_offset_init(void)
{
    zero_offset.w = 1.0f;
    zero_offset.x = 0.0f;
    zero_offset.y = 0.0f;
    zero_offset.z = 0.0f;
    offset_is_set = false;
    
    memset(&calib_stats, 0, sizeof(calib_stats));
    
    LOG_INF("Orientation offset system initialized (Enhanced version)");
    
    /* Try to load saved calibration */
    if (orientation_offset_load() == 0) {
        LOG_INF("Loaded saved orientation offset");
    }
}

int orientation_offset_set_zero(const bno055_quaternion_t *current_quat)
{
    if (!current_quat) {
        return -EINVAL;
    }
    
    if (!quaternion_is_valid(current_quat)) {
        LOG_ERR("Invalid quaternion provided for zero-point");
        return -EINVAL;
    }
    
    /* Calculate the conjugate (inverse) of current orientation
     * This becomes our offset that will transform current orientation to identity */
    quaternion_conjugate(current_quat, &zero_offset);
    quaternion_normalize_precise(&zero_offset);
    
    offset_is_set = true;
    calib_stats.samples_collected = 1;
    calib_stats.drift_corrections = 0;
    
    LOG_INF("Zero orientation set:");
    LOG_INF("  Current: w=%.4f, x=%.4f, y=%.4f, z=%.4f",
            (double)current_quat->w, (double)current_quat->x,
            (double)current_quat->y, (double)current_quat->z);
    LOG_INF("  Offset:  w=%.4f, x=%.4f, y=%.4f, z=%.4f",
            (double)zero_offset.w, (double)zero_offset.x,
            (double)zero_offset.y, (double)zero_offset.z);
    
    /* Save to persistent storage */
    orientation_offset_save();
    
    return 0;
}

int orientation_offset_set_zero_averaged(const bno055_quaternion_t *samples, 
                                         int num_samples)
{
    if (!samples || num_samples <= 0) {
        return -EINVAL;
    }
    bno055_calibration_t calib;
    int ret = bno055_get_calibration(&calib);
    if (ret == 0) {
        /* 只有在充分校准时才允许零位校准 */
        if (calib.gyro < 2) {
            LOG_ERR("Cannot set zero point: insufficient calibration");
            return -EAGAIN;
        }
    }
    
    /* Reset accumulators */
    calib_stats.acc_w = 0.0;
    calib_stats.acc_x = 0.0;
    calib_stats.acc_y = 0.0;
    calib_stats.acc_z = 0.0;
    
    /* Accumulate quaternion components */
    for (int i = 0; i < num_samples; i++) {
        if (!quaternion_is_valid(&samples[i])) {
            LOG_WRN("Skipping invalid sample %d", i);
            continue;
        }
        
        /* For averaging quaternions, we need to ensure they're in the same hemisphere */
        if (i > 0 && samples[i].w * samples[0].w < 0) {
            /* Flip the quaternion if needed */
            calib_stats.acc_w -= samples[i].w;
            calib_stats.acc_x -= samples[i].x;
            calib_stats.acc_y -= samples[i].y;
            calib_stats.acc_z -= samples[i].z;
        } else {
            calib_stats.acc_w += samples[i].w;
            calib_stats.acc_x += samples[i].x;
            calib_stats.acc_y += samples[i].y;
            calib_stats.acc_z += samples[i].z;
        }
    }
    
    /* Calculate average */
    bno055_quaternion_t avg_quat;
    avg_quat.w = (float)(calib_stats.acc_w / num_samples);
    avg_quat.x = (float)(calib_stats.acc_x / num_samples);
    avg_quat.y = (float)(calib_stats.acc_y / num_samples);
    avg_quat.z = (float)(calib_stats.acc_z / num_samples);
    
    /* Normalize the average */
    float norm = sqrtf(avg_quat.w*avg_quat.w + avg_quat.x*avg_quat.x +
                      avg_quat.y*avg_quat.y + avg_quat.z*avg_quat.z);
    avg_quat.w /= norm;
    avg_quat.x /= norm;
    avg_quat.y /= norm;
    avg_quat.z /= norm;
    
    LOG_INF("Averaged zero-point from %d samples", num_samples);
    
    if (fabsf(norm - 1.0f) > 0.01f) {
        LOG_ERR("Averaged quaternion has poor quality (norm=%.4f)", norm);
        return -EINVAL;
    }

    /* Set this as the zero point */
    return orientation_offset_set_zero(&avg_quat);
}

/* 漂移检测阈值 */
#define DRIFT_DETECTION_THRESHOLD  0.1f    /* 四元数norm偏差阈值 */
#define MAX_CONSECUTIVE_DRIFT      5       /* 连续漂移检测次数 */

static uint32_t consecutive_drift_count = 0;

void orientation_offset_apply(const bno055_quaternion_t *raw_quat,
                              bno055_quaternion_t *corrected_quat)
{
    if (!raw_quat || !corrected_quat) {
        LOG_ERR("NULL pointer in orientation_offset_apply");
        return;
    }
    
    if (!offset_is_set) {
        /* No offset set, just copy the raw quaternion */
        corrected_quat->w = raw_quat->w;
        corrected_quat->x = raw_quat->x;
        corrected_quat->y = raw_quat->y;
        corrected_quat->z = raw_quat->z;
        return;
    }
    
    /* Validate input */
    if (!quaternion_is_valid(raw_quat)) {
        LOG_WRN("Invalid raw quaternion, using identity");
        corrected_quat->w = 1.0f;
        corrected_quat->x = 0.0f;
        corrected_quat->y = 0.0f;
        corrected_quat->z = 0.0f;
        return;
    }
    
    /* Apply offset correction: q_corrected = q_offset * q_raw
     * This transforms the raw orientation by the inverse of the zero-point */
    quaternion_multiply_precise(&zero_offset, raw_quat, corrected_quat);
    
    /* 增强漂移检测 */
    if (calib_stats.last_norm_deviation > DRIFT_DETECTION_THRESHOLD) {
        consecutive_drift_count++;
        LOG_WRN("Drift detected (deviation: %.6f, count: %lu)", 
                calib_stats.last_norm_deviation, consecutive_drift_count);
        
        if (consecutive_drift_count >= MAX_CONSECUTIVE_DRIFT) {
            LOG_ERR("Critical drift detected! Resetting orientation system");
            orientation_offset_reset();
            consecutive_drift_count = 0;
            
            /* 强制重新校准 */
            calibrate_zero_point = true; // 需要extern声明
            return;
        }
    } else {
        consecutive_drift_count = 0; // 重置计数器
    }

    /* Periodic drift check and correction */
    calib_stats.samples_collected++;
    if (calib_stats.samples_collected % DRIFT_CORRECTION_INTERVAL == 0) {
        if (calib_stats.last_norm_deviation > QUATERNION_NORM_THRESHOLD) {
            LOG_DBG("Drift detected (deviation: %.6f), applying correction",
                    calib_stats.last_norm_deviation);
            
            /* Re-normalize the offset to prevent drift accumulation */
            quaternion_normalize_precise(&zero_offset);
            calib_stats.drift_corrections++;
            
            if (calib_stats.drift_corrections % 10 == 0) {
                LOG_INF("Applied %u drift corrections", 
                        calib_stats.drift_corrections);
            }
        }
    }
}

void orientation_offset_reset(void)
{
    zero_offset.w = 1.0f;
    zero_offset.x = 0.0f;
    zero_offset.y = 0.0f;
    zero_offset.z = 0.0f;
    offset_is_set = false;
    
    memset(&calib_stats, 0, sizeof(calib_stats));
    
    LOG_INF("Orientation offset reset to identity");
}

bool orientation_offset_is_set(void)
{
    return offset_is_set;
}

int orientation_offset_get_stats(orientation_stats_t *stats)
{
    if (!stats) {
        return -EINVAL;
    }
    
    stats->samples_processed = calib_stats.samples_collected;
    stats->drift_corrections = calib_stats.drift_corrections;
    stats->last_norm_deviation = calib_stats.last_norm_deviation;
    stats->offset_w = zero_offset.w;
    stats->offset_x = zero_offset.x;
    stats->offset_y = zero_offset.y;
    stats->offset_z = zero_offset.z;
    
    return 0;
}

/* Persistent storage implementation using NVS */
#ifdef CONFIG_NVS
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define OFFSET_ID 1
#define OFFSET_MAGIC 0xCAFEBABE  /* Magic number to validate saved data */

static struct nvs_fs fs;
static bool nvs_initialized = false;

typedef struct {
    uint32_t magic;
    quaternion_offset_t offset;
    uint32_t checksum;
} stored_offset_t;

static uint32_t calculate_checksum(const stored_offset_t *data)
{
    uint32_t sum = data->magic;
    sum += (uint32_t)(data->offset.w * 10000);
    sum += (uint32_t)(data->offset.x * 10000);
    sum += (uint32_t)(data->offset.y * 10000);
    sum += (uint32_t)(data->offset.z * 10000);
    return sum;
}

static int init_nvs(void)
{
    if (nvs_initialized) {
        return 0;
    }
    
    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        LOG_ERR("NVS flash device not ready");
        return -ENODEV;
    }
    
    fs.offset = NVS_PARTITION_OFFSET;
    fs.sector_size = 4096;
    fs.sector_count = 2;
    
    int ret = nvs_mount(&fs);
    if (ret) {
        LOG_ERR("NVS mount failed: %d", ret);
        return ret;
    }
    
    nvs_initialized = true;
    LOG_INF("NVS initialized for orientation offset storage");
    return 0;
}

int orientation_offset_save(void)
{
    int ret;
    stored_offset_t data;
    
    if (!offset_is_set) {
        LOG_WRN("No offset to save");
        return -EINVAL;
    }
    
    ret = init_nvs();
    if (ret) {
        return ret;
    }
    
    /* Prepare data for storage */
    data.magic = OFFSET_MAGIC;
    data.offset = zero_offset;
    data.checksum = calculate_checksum(&data);
    
    ret = nvs_write(&fs, OFFSET_ID, &data, sizeof(data));
    if (ret < 0) {
        LOG_ERR("NVS write failed: %d", ret);
        return ret;
    }
    
    LOG_INF("Orientation offset saved to NVS (%.4f, %.4f, %.4f, %.4f)",
            (double)zero_offset.w, (double)zero_offset.x,
            (double)zero_offset.y, (double)zero_offset.z);
    return 0;
}

int orientation_offset_load(void)
{
    int ret;
    stored_offset_t data;
    
    ret = init_nvs();
    if (ret) {
        return ret;
    }
    
    ret = nvs_read(&fs, OFFSET_ID, &data, sizeof(data));
    if (ret < 0) {
        if (ret == -ENOENT) {
            LOG_DBG("No saved offset found");
        } else {
            LOG_ERR("NVS read failed: %d", ret);
        }
        return ret;
    }
    
    /* Validate loaded data */
    if (data.magic != OFFSET_MAGIC) {
        LOG_WRN("Invalid magic number in saved offset");
        return -EINVAL;
    }
    
    uint32_t checksum = calculate_checksum(&data);
    if (checksum != data.checksum) {
        LOG_WRN("Checksum mismatch in saved offset");
        return -EINVAL;
    }
    
    /* Validate quaternion */
    if (!isfinite(data.offset.w) || !isfinite(data.offset.x) ||
        !isfinite(data.offset.y) || !isfinite(data.offset.z)) {
        LOG_WRN("Invalid quaternion values in saved offset");
        return -EINVAL;
    }
    
    /* Apply loaded offset */
    zero_offset = data.offset;
    quaternion_normalize_precise(&zero_offset);
    offset_is_set = true;
    
    LOG_INF("Orientation offset loaded from NVS (%.4f, %.4f, %.4f, %.4f)",
            (double)zero_offset.w, (double)zero_offset.x,
            (double)zero_offset.y, (double)zero_offset.z);
    return 0;
}

int orientation_offset_clear_storage(void)
{
    int ret = init_nvs();
    if (ret) {
        return ret;
    }
    
    ret = nvs_delete(&fs, OFFSET_ID);
    if (ret < 0 && ret != -ENOENT) {
        LOG_ERR("Failed to clear stored offset: %d", ret);
        return ret;
    }
    
    LOG_INF("Cleared stored orientation offset");
    return 0;
}

#else
/* Stub implementations when NVS is not available */
int orientation_offset_save(void) 
{ 
    LOG_WRN("NVS not configured, cannot save offset");
    return -ENOTSUP; 
}

int orientation_offset_load(void) 
{ 
    LOG_DBG("NVS not configured, cannot load offset");
    return -ENOTSUP; 
}

int orientation_offset_clear_storage(void)
{
    LOG_WRN("NVS not configured");
    return -ENOTSUP;
}
#endif /* CONFIG_NVS */