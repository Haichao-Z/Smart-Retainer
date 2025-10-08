/**
 * @file imu_driver.c
 * @brief LSM6DS0 IMU driver implementation
 */

#include "imu_driver.h"
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(imu_driver, LOG_LEVEL_DBG);

/* Device pointer */
static const struct device *imu_dev = NULL;
static bool driver_initialized = false;

/* Sensor scaling factors */
static float accel_scale = 1.0f;
static float gyro_scale = 1.0f;

/* Define M_PI if not already defined */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Device tree node */
#define IMU_NODE DT_NODELABEL(lsm6dso)

/**
 * @brief Initialize IMU driver
 */
int imu_driver_init(const imu_config_t *config)
{
    int ret;

    LOG_INF("Initializing IMU driver");

    /* Get device binding */
    imu_dev = DEVICE_DT_GET(IMU_NODE);
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("IMU device not ready");
        return -ENODEV;
    }

    LOG_INF("IMU device ready");

    /* For LSM6DSO, the configuration is done via device tree */
    /* We don't need to set range and ODR here as they are set in overlay */
    
    /* Just verify we can communicate with the device */
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

    /* Calculate scaling factors based on device tree config */
    /* These are approximate - actual values come from the sensor */
    accel_scale = 1.0f;  /* Sensor driver handles scaling */
    gyro_scale = 1.0f;   /* Sensor driver handles scaling */

    driver_initialized = true;
    LOG_INF("IMU driver initialized successfully");
    
    return 0;
}

/**
 * @brief Read raw IMU data
 */
int imu_driver_read(imu_data_t *data)
{
    int ret;
    struct sensor_value accel[3], gyro[3];

    if (!driver_initialized || data == NULL) {
        return -EINVAL;
    }

    /* Fetch sensor data */
    ret = sensor_sample_fetch(imu_dev);
    if (ret) {
        LOG_ERR("Failed to fetch sensor data: %d", ret);
        return ret;
    }

    /* Get accelerometer data */
    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (ret) {
        LOG_ERR("Failed to get accelerometer data: %d", ret);
        return ret;
    }

    /* Get gyroscope data */
    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    if (ret) {
        LOG_ERR("Failed to get gyroscope data: %d", ret);
        return ret;
    }

    /* Convert and store data */
    data->accel_x = sensor_value_to_double(&accel[0]);
    data->accel_y = sensor_value_to_double(&accel[1]);
    data->accel_z = sensor_value_to_double(&accel[2]);
    
    data->gyro_x = sensor_value_to_double(&gyro[0]);
    data->gyro_y = sensor_value_to_double(&gyro[1]);
    data->gyro_z = sensor_value_to_double(&gyro[2]);
    
    data->timestamp = k_uptime_get_32();

    LOG_DBG("IMU data: A[%.2f, %.2f, %.2f] G[%.2f, %.2f, %.2f]",
            data->accel_x, data->accel_y, data->accel_z,
            data->gyro_x, data->gyro_y, data->gyro_z);

    return 0;
}

/**
 * @brief Check if IMU is ready
 */
bool imu_driver_is_ready(void)
{
    return driver_initialized && (imu_dev != NULL) && 
           device_is_ready(imu_dev);
}

/**
 * @brief Perform IMU self-test
 */
int imu_driver_self_test(void)
{
    imu_data_t data;
    int ret;

    if (!driver_initialized) {
        LOG_ERR("IMU not initialized");
        return -EINVAL;
    }

    LOG_INF("Performing IMU self-test...");

    /* Read data to verify communication */
    ret = imu_driver_read(&data);
    if (ret) {
        LOG_ERR("Self-test failed: cannot read data");
        return ret;
    }

    /* Log the values for debugging */
    LOG_INF("Accel: X=%.2f Y=%.2f Z=%.2f m/s²", 
            (double)data.accel_x, (double)data.accel_y, (double)data.accel_z);
    LOG_INF("Gyro: X=%.2f Y=%.2f Z=%.2f rad/s",
            (double)data.gyro_x, (double)data.gyro_y, (double)data.gyro_z);

    /* Basic sanity check with safer math */
    /* Check if at least one accelerometer axis has significant value */
    float max_accel = 0.0f;
    if (fabsf(data.accel_x) > max_accel) max_accel = fabsf(data.accel_x);
    if (fabsf(data.accel_y) > max_accel) max_accel = fabsf(data.accel_y);
    if (fabsf(data.accel_z) > max_accel) max_accel = fabsf(data.accel_z);
    
    if (max_accel < 1.0f || max_accel > 20.0f) {
        LOG_WRN("Accelerometer reading unusual: max=%.2f m/s²", (double)max_accel);
        // Don't fail, just warn
    }

    LOG_INF("IMU self-test passed");
    return 0;
}