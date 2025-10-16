/**
 * @file orientation_offset.h
 * @brief 姿态零点校准 - 解决初始姿态偏移问题
 */

#ifndef ORIENTATION_OFFSET_H
#define ORIENTATION_OFFSET_H

#include <zephyr/kernel.h>
#include "bno055_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 四元数结构（用于存储偏移量）
 */
typedef struct {
    float w, x, y, z;
} quaternion_offset_t;

/**
 * @brief 初始化姿态偏移系统
 */
void orientation_offset_init(void);

/**
 * @brief 设置当前姿态为零点
 * 
 * 用户佩戴好牙套并摆正姿势后调用此函数
 * 
 * @param current_quat 当前的四元数姿态
 * @return 0 成功, 负数失败
 */
int orientation_offset_set_zero(const bno055_quaternion_t *current_quat);

/**
 * @brief 应用偏移校正到姿态数据
 * 
 * @param raw_quat 原始四元数（从IMU读取）
 * @param corrected_quat 校正后的四元数（输出）
 */
void orientation_offset_apply(const bno055_quaternion_t *raw_quat, 
                              bno055_quaternion_t *corrected_quat);

/**
 * @brief 重置偏移量（恢复到单位四元数）
 */
void orientation_offset_reset(void);

/**
 * @brief 检查是否已设置偏移量
 */
bool orientation_offset_is_set(void);

/**
 * @brief 保存偏移量到持久化存储（可选）
 */
int orientation_offset_save(void);

/**
 * @brief 从持久化存储加载偏移量（可选）
 */
int orientation_offset_load(void);

#ifdef __cplusplus
}
#endif

#endif /* ORIENTATION_OFFSET_H */