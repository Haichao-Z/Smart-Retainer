/**
 * @file imu_driver.c (COORDINATE FIX SOLUTION)
 * @brief Corrected coordinate mapping for consistent rotation behavior
 * 
 * PROBLEM ANALYSIS:
 * ================
 * Physical sensor orientation (board held vertically facing user):
 *   Sensor PCB marking: X=Right, Y=Up, Z=Forward
 *   Gravity direction: -Y (pointing down, so Y reads ~-9.8 m/s²)
 * 
 * User's expected coordinate system (board vertical):
 *   User_X = Right
 *   User_Y = Up  
 *   User_Z = Forward (toward user)
 * 
 * Web visualization coordinate system (Three.js standard):
 *   Web_X = Right (Red axis)
 *   Web_Y = Up (Green axis)
 *   Web_Z = Forward toward camera (Blue axis)
 * 
 * SOLUTION:
 * =========
 * Since the physical sensor axes ALREADY match our desired coordinate system,
 * we only need to ensure Madgwick gets gravity on the correct axis.
 * 
 * Strategy: Apply MINIMAL transformation
 * - Keep X, Y, Z axes as-is (they already match user expectations)
 * - Invert Y to make gravity positive (Madgwick expects +Z = up with +gravity)
 * - Actually, better approach: Let Madgwick handle gravity in -Y naturally
 * 
 * BETTER STRATEGY: No axis remapping, just sign correction
 * ========================================================
 * The root cause was the previous code mapped:
 *   Physical Z → Algorithm Y (wrong!)
 *   Physical Y → Algorithm Z
 * 
 * This caused:
 *   Physical rotation around Z → Algorithm sees rotation around Y
 * 
 * NEW MAPPING (Identity + sign adjustments only):
 * ===============================================
 * Output_X = Sensor_X (right is right)
 * Output_Y = Sensor_Y (up is up, gravity is -Y)  
 * Output_Z = Sensor_Z (forward is forward)
 * 
 * For Madgwick compatibility when board is vertical:
 * - Gravity is on -Y axis (~-9.8 m/s²)
 * - Madgwick should be initialized to expect this
 * - Roll=0, Pitch=0, Yaw=0 when board faces user upright
 */

#include "imu_driver.h"
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(imu_driver, LOG_LEVEL_DBG);

static const struct device *imu_dev = NULL;
static bool driver_initialized = false;

#define IMU_NODE DT_NODELABEL(lsm6dso)

/**
 * @brief Apply coordinate transformation (CORRECTED VERSION)
 * 
 * Key insight: Keep axes aligned with physical labels
 * - X remains X (right)
 * - Y remains Y (up, gravity is -Y)
 * - Z remains Z (forward)
 * 
 * This ensures physical rotation around Z axis = mathematical rotation around Z
 */
static void apply_coordinate_correction(float *x, float *y, float *z)
{
    // Read raw sensor values
    float sensor_x = *x;  /* Right (per PCB marking) */
    float sensor_y = *y;  /* Up (per PCB marking, gravity = -Y) */
    float sensor_z = *z;  /* Forward (per PCB marking) */
    
    /* IDENTITY MAPPING: Keep coordinate system as-is
     * This maintains physical correspondence:
     * - Physical X → Output X (right)
     * - Physical Y → Output Y (up)  
     * - Physical Z → Output Z (forward)
     * 
     * Result: Physical Z-axis rotation = Output Z-axis rotation ✓
     */
    *x = sensor_x;   /* Right */
    *y = sensor_y;   /* Up (gravity is -Y) */
    *z = sensor_z;   /* Forward */
    
    /* Alternative if you need right-handed system with +Z up:
     * Uncomment this section if Madgwick absolutely requires gravity on +Z
     * 
     * *x = sensor_x;    // Right stays right
     * *y = sensor_z;    // Forward → Y  
     * *z = -sensor_y;   // Up → Z (negate to make gravity positive)
     * 
     * But this will change the rotation axis mapping!
     * Better to keep identity and configure Madgwick appropriately.
     */
}

int imu_driver_init(const imu_config_t *config)
{
    int ret;

    LOG_INF("Initializing IMU driver");
    LOG_INF("Coordinate mapping: Identity (X=X, Y=Y, Z=Z)");
    LOG_INF("Expected: X=Right, Y=Up(gravity=-Y), Z=Forward");

    imu_dev = DEVICE_DT_GET(IMU_NODE);
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("IMU device not ready");
        return -ENODEV;
    }

    LOG_INF("IMU device ready");

    /* Test sensor communication */
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
    
    /* Apply coordinate correction (identity mapping) */
    float ax = ax_raw;
    float ay = ay_raw;
    float az = az_raw;
    apply_coordinate_correction(&ax, &ay, &az);
    
    float gx = gx_raw;
    float gy = gy_raw;
    float gz = gz_raw;
    apply_coordinate_correction(&gx, &gy, &gz);
    
    /* Store corrected data */
    data->accel_x = ax;
    data->accel_y = ay;
    data->accel_z = az;
    
    data->gyro_x = gx;
    data->gyro_y = gy;
    data->gyro_z = gz;
    
    data->timestamp = k_uptime_get_32();

    LOG_DBG("IMU Data: A[%.2f, %.2f, %.2f] G[%.2f, %.2f, %.2f]",
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

    LOG_INF("=== IMU Self-Test (Identity Coordinate Mapping) ===");
    LOG_INF("Coordinate system: X=Right, Y=Up, Z=Forward");
    LOG_INF("Expected when vertical: Y ≈ -9.8 m/s² (gravity down)");

    ret = imu_driver_read(&data);
    if (ret) {
        LOG_ERR("Self-test failed: cannot read data");
        return ret;
    }

    LOG_INF("\nAccelerometer:");
    LOG_INF("  X = %.2f m/s² (right/left)", (double)data.accel_x);
    LOG_INF("  Y = %.2f m/s² (up/down)", (double)data.accel_y);
    LOG_INF("  Z = %.2f m/s² (forward/back)", (double)data.accel_z);
    
    LOG_INF("Gyroscope:");
    LOG_INF("  X = %.2f rad/s (roll around X-axis)", (double)data.gyro_x);
    LOG_INF("  Y = %.2f rad/s (pitch around Y-axis)", (double)data.gyro_y);
    LOG_INF("  Z = %.2f rad/s (yaw around Z-axis)", (double)data.gyro_z);

    /* Verify gravity direction */
    LOG_INF("\n=== Gravity Verification ===");
    LOG_INF("When board is VERTICAL (facing user):");
    LOG_INF("  Expected: Y ≈ -9.8 m/s² (gravity pointing down)");
    LOG_INF("  Actual:   Y = %.2f m/s²", (double)data.accel_y);
    
    float abs_y = fabsf(data.accel_y);
    if (abs_y > 8.0f && abs_y < 11.0f) {
        LOG_INF("✓ SUCCESS: Gravity detected on Y-axis");
        if (data.accel_y < 0) {
            LOG_INF("✓ Gravity direction correct (pointing down = -Y)");
        }
    } else {
        LOG_WRN("✗ UNEXPECTED: Y = %.2f (expected ~±9.8)", (double)data.accel_y);
        LOG_WRN("  X = %.2f, Z = %.2f", (double)data.accel_x, (double)data.accel_z);
    }

    float mag = sqrtf(data.accel_x * data.accel_x + 
                      data.accel_y * data.accel_y + 
                      data.accel_z * data.accel_z);
    LOG_INF("Acceleration magnitude: %.2f m/s²", (double)mag);

    LOG_INF("\n=== Expected Rotation Behavior ===");
    LOG_INF("Physical Z-axis rotation (board facing you, rotate like steering wheel):");
    LOG_INF("  → Gyro Z should change");
    LOG_INF("  → Yaw angle should change");
    LOG_INF("  → Web model should rotate around its Z-axis (blue)");
    LOG_INF("\nTilt board RIGHT → Roll changes (rotation around X, red axis)");
    LOG_INF("Tilt board AWAY → Pitch changes (rotation around Y, green axis)");

    LOG_INF("\n=== Self-Test Complete ===\n");
    return 0;
}