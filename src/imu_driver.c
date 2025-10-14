/**
 * @file imu_driver.c (FINAL SOLUTION)
 * @brief Correct coordinate mapping for Madgwick compatibility
 * 
 * Problem: Madgwick assumes gravity on Z-axis, but physically gravity is on Y-axis
 * 
 * Sensor physical orientation (board vertical, facing user):
 *   Sensor_X = Left  (horizontal)
 *   Sensor_Y = Up    (has gravity!)
 *   Sensor_Z = Forward (horizontal)
 * 
 * User's expected coordinate system (board vertical):
 *   User_X = Right
 *   User_Y = Up
 *   User_Z = Forward
 * 
 * Madgwick's required coordinate system (for Roll=0):
 *   Algorithm expects gravity on Z-axis!
 *   So we need to remap to make Z-axis vertical
 * 
 * Solution: Map coordinates so Madgwick sees gravity on Z
 *   Output_X (Right) = -Sensor_X (Left negated)
 *   Output_Y (Forward) = Sensor_Z (was forward)
 *   Output_Z (Up) = Sensor_Y (was up, has gravity)
 */

#include "imu_driver.h"
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(imu_driver, LOG_LEVEL_DBG);

static const struct device *imu_dev = NULL;
static bool driver_initialized = false;

#define IMU_NODE DT_NODELABEL(lsm6dso)

/**
 * @brief Apply coordinate transformation for Madgwick compatibility
 * 
 * Remap axes so that:
 * - Gravity appears on Z-axis (required by Madgwick)
 * - X-axis points right
 * - Y-axis points forward
 * - Z-axis points up
 */
static void apply_madgwick_transform(float *x, float *y, float *z)
{
    float sensor_x = *x;  /* Left (horizontal) */
    float sensor_y = *y;  /* Up (has gravity ~9.8) */
    float sensor_z = *z;  /* Forward (horizontal) */
    
    /* Transform to make gravity on Z-axis for Madgwick */
    *x = -sensor_x;       /* Left → Right (negate) */
    *y = sensor_z;        /* Original forward → Algorithm forward */
    *z = sensor_y;        /* Original up (gravity) → Algorithm up (gravity) */
    
    /* Now Madgwick will see:
     * - Z has gravity (what it expects)
     * - X points right
     * - Y points forward
     * - When board is vertical: Roll=0, Pitch=0, Yaw=0 ✓
     */
}

int imu_driver_init(const imu_config_t *config)
{
    int ret;

    LOG_INF("Initializing IMU driver");
    LOG_INF("Applying Madgwick-compatible coordinate transform");

    imu_dev = DEVICE_DT_GET(IMU_NODE);
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("IMU device not ready");
        return -ENODEV;
    }

    LOG_INF("IMU device ready");

    struct sensor_value test_val;
    ret = sensor_sample_fetch(imu_dev);
    if (ret) {
        LOG_ERR("Failed to fetch initial sample: %d", ret);
        return ret;
    }

    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, &test_val);
    if (ret) {
        LOG_ERR("Failed to get accelerometer data: %d", ret);
        return ret;
    }

    driver_initialized = true;
    LOG_INF("IMU driver initialized successfully");
    
    return 0;
}

int imu_driver_read(imu_data_t *data)
{
    int ret;
    struct sensor_value accel[3], gyro[3];

    if (!driver_initialized || data == NULL) {
        return -EINVAL;
    }

    ret = sensor_sample_fetch(imu_dev);
    if (ret) {
        LOG_ERR("Failed to fetch sensor data: %d", ret);
        return ret;
    }

    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (ret) {
        LOG_ERR("Failed to get accelerometer data: %d", ret);
        return ret;
    }

    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    if (ret) {
        LOG_ERR("Failed to get gyroscope data: %d", ret);
        return ret;
    }

    /* Read raw sensor values */
    float ax_raw = sensor_value_to_double(&accel[0]);
    float ay_raw = sensor_value_to_double(&accel[1]);
    float az_raw = sensor_value_to_double(&accel[2]);
    
    float gx_raw = sensor_value_to_double(&gyro[0]);
    float gy_raw = sensor_value_to_double(&gyro[1]);
    float gz_raw = sensor_value_to_double(&gyro[2]);
    
    /* Apply Madgwick-compatible transform */
    float ax = ax_raw;
    float ay = ay_raw;
    float az = az_raw;
    apply_madgwick_transform(&ax, &ay, &az);
    
    float gx = gx_raw;
    float gy = gy_raw;
    float gz = gz_raw;
    apply_madgwick_transform(&gx, &gy, &gz);
    
    /* Store transformed data */
    data->accel_x = ax;
    data->accel_y = ay;
    data->accel_z = az;
    
    data->gyro_x = gx;
    data->gyro_y = gy;
    data->gyro_z = gz;
    
    data->timestamp = k_uptime_get_32();

    LOG_DBG("Raw:   A[%.2f, %.2f, %.2f] G[%.2f, %.2f, %.2f]",
            ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);
    LOG_DBG("Trans: A[%.2f, %.2f, %.2f] G[%.2f, %.2f, %.2f]",
            ax, ay, az, gx, gy, gz);

    return 0;
}

bool imu_driver_is_ready(void)
{
    return driver_initialized && (imu_dev != NULL) && 
           device_is_ready(imu_dev);
}

int imu_driver_self_test(void)
{
    imu_data_t data;
    int ret;

    if (!driver_initialized) {
        LOG_ERR("IMU not initialized");
        return -EINVAL;
    }

    LOG_INF("=== IMU Self-Test (Madgwick Transform) ===");
    LOG_INF("Transform: X=-X, Y=Z, Z=Y (gravity on Z for Madgwick)");

    ret = imu_driver_read(&data);
    if (ret) {
        LOG_ERR("Self-test failed: cannot read data");
        return ret;
    }

    LOG_INF("\nAccelerometer (transformed for Madgwick):");
    LOG_INF("  X = %.2f m/s² (right/left)", (double)data.accel_x);
    LOG_INF("  Y = %.2f m/s² (forward/back)", (double)data.accel_y);
    LOG_INF("  Z = %.2f m/s² (up/down)", (double)data.accel_z);
    
    LOG_INF("Gyroscope (transformed):");
    LOG_INF("  X = %.2f rad/s (roll rate)", (double)data.gyro_x);
    LOG_INF("  Y = %.2f rad/s (pitch rate)", (double)data.gyro_y);
    LOG_INF("  Z = %.2f rad/s (yaw rate)", (double)data.gyro_z);

    /* Verify gravity on Z-axis */
    LOG_INF("\n=== Gravity Verification ===");
    LOG_INF("When board is VERTICAL (facing user):");
    LOG_INF("  Expected: Z ≈ +9.8 m/s² (Madgwick standard)");
    LOG_INF("  Actual:   Z = %.2f m/s²", (double)data.accel_z);
    
    float abs_z = fabsf(data.accel_z);
    if (abs_z > 8.0f && abs_z < 11.0f) {
        LOG_INF("✓ SUCCESS: Gravity on Z-axis!");
        LOG_INF("✓ Madgwick will compute Roll≈0° for this orientation!");
    } else {
        LOG_WRN("✗ PROBLEM: Z = %.2f (expected ~9.8)", (double)data.accel_z);
        LOG_WRN("  X = %.2f, Y = %.2f", (double)data.accel_x, (double)data.accel_y);
    }

    float mag = sqrtf(data.accel_x * data.accel_x + 
                      data.accel_y * data.accel_y + 
                      data.accel_z * data.accel_z);
    LOG_INF("Acceleration magnitude: %.2f m/s²", (double)mag);

    LOG_INF("\n=== Expected Behavior ===");
    LOG_INF("Board vertical (facing you) → Roll≈0°, Pitch≈0°, Yaw≈0°");
    LOG_INF("Tilt board RIGHT → Roll increases");
    LOG_INF("Tilt board AWAY from you → Pitch increases");  
    LOG_INF("Rotate board CW (your view) → Yaw increases");

    LOG_INF("\n=== Self-Test Complete ===\n");
    return 0;
}