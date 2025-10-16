#include <math.h>
#include <zephyr/logging/log.h>
#include "orientation_offset.h"

LOG_MODULE_REGISTER(orientation_offset, LOG_LEVEL_DBG);

/* 存储零点偏移的四元数（逆四元数） */
static quaternion_offset_t zero_offset = {1.0f, 0.0f, 0.0f, 0.0f};
static bool offset_is_set = false;

/**
 * @brief 四元数共轭（用于求逆）
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
 * @brief 四元数归一化
 */
static void quaternion_normalize(quaternion_offset_t *q)
{
    float norm = sqrtf(q->w * q->w + q->x * q->x + 
                      q->y * q->y + q->z * q->z);
    
    if (norm > 0.0001f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    } else {
        /* 如果范数太小，重置为单位四元数 */
        q->w = 1.0f;
        q->x = q->y = q->z = 0.0f;
    }
}

/**
 * @brief 四元数乘法 (q1 * q2)
 * 
 * 用于组合旋转：result = q_offset_inv * q_current
 */
static void quaternion_multiply(const quaternion_offset_t *q1,
                                const bno055_quaternion_t *q2,
                                bno055_quaternion_t *result)
{
    result->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void orientation_offset_init(void)
{
    zero_offset.w = 1.0f;
    zero_offset.x = 0.0f;
    zero_offset.y = 0.0f;
    zero_offset.z = 0.0f;
    offset_is_set = false;
    
    LOG_INF("Orientation offset system initialized");
}

int orientation_offset_set_zero(const bno055_quaternion_t *current_quat)
{
    if (!current_quat) {
        return -EINVAL;
    }
    
    /* 计算当前四元数的共轭（逆旋转）
     * 这样后续乘以这个逆四元数就能抵消初始偏移 */
    quaternion_conjugate(current_quat, &zero_offset);
    quaternion_normalize(&zero_offset);
    
    offset_is_set = true;
    
    LOG_INF("Zero orientation set:");
    LOG_INF("  Offset quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f",
            (double)zero_offset.w, (double)zero_offset.x,
            (double)zero_offset.y, (double)zero_offset.z);
    
    return 0;
}

void orientation_offset_apply(const bno055_quaternion_t *raw_quat,
                              bno055_quaternion_t *corrected_quat)
{
    if (!raw_quat || !corrected_quat) {
        return;
    }
    
    if (!offset_is_set) {
        /* 没有设置偏移量，直接复制 */
        corrected_quat->w = raw_quat->w;
        corrected_quat->x = raw_quat->x;
        corrected_quat->y = raw_quat->y;
        corrected_quat->z = raw_quat->z;
        return;
    }
    
    /* 应用偏移校正：q_corrected = q_offset_inv * q_raw
     * 这会将"零点姿态"变成单位四元数（无旋转） */
    quaternion_multiply(&zero_offset, raw_quat, corrected_quat);
}

void orientation_offset_reset(void)
{
    zero_offset.w = 1.0f;
    zero_offset.x = 0.0f;
    zero_offset.y = 0.0f;
    zero_offset.z = 0.0f;
    offset_is_set = false;
    
    LOG_INF("Orientation offset reset");
}

bool orientation_offset_is_set(void)
{
    return offset_is_set;
}

/* 持久化存储部分（可选，使用NVS） */
#ifdef CONFIG_NVS
#include <zephyr/fs/nvs.h>

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define OFFSET_ID 1

static struct nvs_fs fs;

int orientation_offset_save(void)
{
    int ret;
    
    struct nvs_fs *fs_ptr = &fs;
    fs_ptr->flash_device = NVS_PARTITION_DEVICE;
    fs_ptr->offset = NVS_PARTITION_OFFSET;
    fs_ptr->sector_size = 4096;
    fs_ptr->sector_count = 2;
    
    ret = nvs_mount(fs_ptr);
    if (ret) {
        LOG_ERR("NVS mount failed: %d", ret);
        return ret;
    }
    
    ret = nvs_write(fs_ptr, OFFSET_ID, &zero_offset, sizeof(zero_offset));
    if (ret < 0) {
        LOG_ERR("NVS write failed: %d", ret);
        return ret;
    }
    
    LOG_INF("Orientation offset saved to NVS");
    return 0;
}

int orientation_offset_load(void)
{
    int ret;
    
    struct nvs_fs *fs_ptr = &fs;
    fs_ptr->flash_device = NVS_PARTITION_DEVICE;
    fs_ptr->offset = NVS_PARTITION_OFFSET;
    fs_ptr->sector_size = 4096;
    fs_ptr->sector_count = 2;
    
    ret = nvs_mount(fs_ptr);
    if (ret) {
        LOG_ERR("NVS mount failed: %d", ret);
        return ret;
    }
    
    ret = nvs_read(fs_ptr, OFFSET_ID, &zero_offset, sizeof(zero_offset));
    if (ret < 0) {
        LOG_WRN("No saved offset found");
        return ret;
    }
    
    offset_is_set = true;
    LOG_INF("Orientation offset loaded from NVS");
    return 0;
}
#else
int orientation_offset_save(void) { return -ENOTSUP; }
int orientation_offset_load(void) { return -ENOTSUP; }
#endif