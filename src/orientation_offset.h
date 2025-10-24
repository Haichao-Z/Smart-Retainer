/**
 * @file orientation_offset.h
 * @brief Enhanced orientation offset management header
 */

#ifndef ORIENTATION_OFFSET_H
#define ORIENTATION_OFFSET_H

#include <stdbool.h>
#include <stdint.h>
#include "bno055_driver.h"

/* Offset quaternion type (same as BNO055 quaternion) */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_offset_t;

/* Statistics structure for monitoring */
typedef struct {
    uint32_t samples_processed;
    uint32_t drift_corrections;
    float last_norm_deviation;
    float offset_w;
    float offset_x;
    float offset_y;
    float offset_z;
} orientation_stats_t;

/**
 * @brief Initialize the orientation offset system
 */
void orientation_offset_init(void);

/**
 * @brief Set current orientation as zero reference point
 * 
 * @param current_quat Current quaternion from BNO055
 * @return 0 on success, negative error code on failure
 */
int orientation_offset_set_zero(const bno055_quaternion_t *current_quat);

/**
 * @brief Set zero reference using averaged samples
 * 
 * @param samples Array of quaternion samples
 * @param num_samples Number of samples to average
 * @return 0 on success, negative error code on failure
 */
int orientation_offset_set_zero_averaged(const bno055_quaternion_t *samples, 
                                         int num_samples);

/**
 * @brief Apply offset correction to raw quaternion
 * 
 * @param raw_quat Input quaternion from sensor
 * @param corrected_quat Output corrected quaternion
 */
void orientation_offset_apply(const bno055_quaternion_t *raw_quat,
                              bno055_quaternion_t *corrected_quat);

/**
 * @brief Reset offset to identity (no correction)
 */
void orientation_offset_reset(void);

/**
 * @brief Check if offset has been set
 * 
 * @return true if offset is set, false otherwise
 */
bool orientation_offset_is_set(void);

/**
 * @brief Get statistics about offset operation
 * 
 * @param stats Pointer to statistics structure
 * @return 0 on success, negative error code on failure
 */
int orientation_offset_get_stats(orientation_stats_t *stats);

/**
 * @brief Save current offset to persistent storage
 * 
 * @return 0 on success, negative error code on failure
 */
int orientation_offset_save(void);

/**
 * @brief Load saved offset from persistent storage
 * 
 * @return 0 on success, negative error code on failure
 */
int orientation_offset_load(void);

/**
 * @brief Clear saved offset from storage
 * 
 * @return 0 on success, negative error code on failure
 */
int orientation_offset_clear_storage(void);

#endif /* ORIENTATION_OFFSET_H */