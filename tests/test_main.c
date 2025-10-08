/**
 * @file test_main.c
 * @brief Unit tests for Smart Retainer modules
 * 
 * Comprehensive testing for IMU driver, attitude fusion, and BLE service
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <math.h>

#include "imu_driver.h"
#include "attitude_fusion.h"
#include "ble_imu_service.h"

/* Test tolerance for floating point comparisons */
#define FLOAT_TOLERANCE 0.001f

/* Helper function to compare floats */
static bool float_equal(float a, float b, float tolerance)
{
    return fabsf(a - b) < tolerance;
}

/**
 * Test Suite 1: IMU Driver Tests
 */
ZTEST_SUITE(imu_driver_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(imu_driver_tests, test_imu_init_valid_config)
{
    imu_config_t config = {
        .sample_rate_hz = 100,
        .accel_range_g = 2,
        .gyro_range_dps = 250
    };

    int ret = imu_driver_init(&config);
    zassert_equal(ret, 0, "IMU initialization should succeed with valid config");
    zassert_true(imu_driver_is_ready(), "IMU should be ready after init");
}

ZTEST(imu_driver_tests, test_imu_init_invalid_config)
{
    /* Test with NULL config */
    int ret = imu_driver_init(NULL);
    zassert_not_equal(ret, 0, "IMU init should fail with NULL config");
}

ZTEST(imu_driver_tests, test_imu_read_data)
{
    imu_data_t data;
    
    int ret = imu_driver_read(&data);
    zassert_equal(ret, 0, "IMU read should succeed");
    
    /* Check that data is within reasonable ranges */
    /* Accelerometer: ±2g = ±19.6 m/s² */
    zassert_true(fabsf(data.accel_x) < 20.0f, "Accel X out of range");
    zassert_true(fabsf(data.accel_y) < 20.0f, "Accel Y out of range");
    zassert_true(fabsf(data.accel_z) < 20.0f, "Accel Z out of range");
    
    /* Gyroscope: ±250 dps = ±4.36 rad/s */
    zassert_true(fabsf(data.gyro_x) < 5.0f, "Gyro X out of range");
    zassert_true(fabsf(data.gyro_y) < 5.0f, "Gyro Y out of range");
    zassert_true(fabsf(data.gyro_z) < 5.0f, "Gyro Z out of range");
    
    /* Timestamp should be non-zero */
    zassert_not_equal(data.timestamp, 0, "Timestamp should be non-zero");
}

ZTEST(imu_driver_tests, test_imu_self_test)
{
    int ret = imu_driver_self_test();
    zassert_equal(ret, 0, "IMU self-test should pass");
}

ZTEST(imu_driver_tests, test_imu_read_null_pointer)
{
    int ret = imu_driver_read(NULL);
    zassert_not_equal(ret, 0, "IMU read should fail with NULL pointer");
}

/**
 * Test Suite 2: Attitude Fusion Tests
 */
ZTEST_SUITE(attitude_fusion_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(attitude_fusion_tests, test_attitude_init)
{
    int ret = attitude_fusion_init(0.1f, 0.01f);
    zassert_equal(ret, 0, "Attitude fusion init should succeed");
}

ZTEST(attitude_fusion_tests, test_attitude_init_invalid_params)
{
    int ret;
    
    /* Test with negative beta */
    ret = attitude_fusion_init(-0.1f, 0.01f);
    zassert_not_equal(ret, 0, "Init should fail with negative beta");
    
    /* Test with zero sample period */
    ret = attitude_fusion_init(0.1f, 0.0f);
    zassert_not_equal(ret, 0, "Init should fail with zero sample period");
}

ZTEST(attitude_fusion_tests, test_quaternion_normalization)
{
    /* Initialize fusion */
    attitude_fusion_init(0.1f, 0.01f);
    
    /* Create test IMU data (stationary, Z-up) */
    imu_data_t imu_data = {
        .accel_x = 0.0f,
        .accel_y = 0.0f,
        .accel_z = 9.81f,
        .gyro_x = 0.0f,
        .gyro_y = 0.0f,
        .gyro_z = 0.0f,
        .timestamp = k_uptime_get_32()
    };
    
    attitude_t attitude;
    int ret = attitude_fusion_update(&imu_data, &attitude);
    zassert_equal(ret, 0, "Attitude update should succeed");
    
    /* Check quaternion normalization */
    float quat_norm = sqrtf(
        attitude.quaternion.w * attitude.quaternion.w +
        attitude.quaternion.x * attitude.quaternion.x +
        attitude.quaternion.y * attitude.quaternion.y +
        attitude.quaternion.z * attitude.quaternion.z
    );
    
    zassert_true(float_equal(quat_norm, 1.0f, FLOAT_TOLERANCE),
                 "Quaternion should be normalized");
}

ZTEST(attitude_fusion_tests, test_euler_conversion)
{
    quaternion_t q;
    euler_angles_t euler;
    
    /* Test 1: Identity quaternion should give zero Euler angles */
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    
    quaternion_to_euler(&q, &euler);
    
    zassert_true(float_equal(euler.roll, 0.0f, FLOAT_TOLERANCE),
                 "Roll should be zero for identity");
    zassert_true(float_equal(euler.pitch, 0.0f, FLOAT_TOLERANCE),
                 "Pitch should be zero for identity");
    zassert_true(float_equal(euler.yaw, 0.0f, FLOAT_TOLERANCE),
                 "Yaw should be zero for identity");
    
    /* Test 2: 90-degree roll rotation */
    q.w = 0.7071f;  /* cos(45°) */
    q.x = 0.7071f;  /* sin(45°) */
    q.y = 0.0f;
    q.z = 0.0f;
    
    quaternion_to_euler(&q, &euler);
    
    /* 90-degree rotation = π/2 radians */
    zassert_true(float_equal(euler.roll, M_PI / 2.0f, 0.1f),
                 "Roll should be 90 degrees");
}

ZTEST(attitude_fusion_tests, test_attitude_reset)
{
    attitude_fusion_init(0.1f, 0.01f);
    attitude_fusion_reset();
    
    attitude_t attitude;
    int ret = attitude_fusion_get_current(&attitude);
    zassert_equal(ret, 0, "Get current attitude should succeed");
    
    /* After reset, quaternion should be identity */
    zassert_true(float_equal(attitude.quaternion.w, 1.0f, FLOAT_TOLERANCE),
                 "Quaternion w should be 1 after reset");
    zassert_true(float_equal(attitude.quaternion.x, 0.0f, FLOAT_TOLERANCE),
                 "Quaternion x should be 0 after reset");
}

ZTEST(attitude_fusion_tests, test_attitude_update_null_pointers)
{
    attitude_fusion_init(0.1f, 0.01f);
    
    imu_data_t imu_data;
    attitude_t attitude;
    int ret;
    
    /* Test with NULL IMU data */
    ret = attitude_fusion_update(NULL, &attitude);
    zassert_not_equal(ret, 0, "Update should fail with NULL IMU data");
    
    /* Test with NULL attitude output */
    ret = attitude_fusion_update(&imu_data, NULL);
    zassert_not_equal(ret, 0, "Update should fail with NULL attitude");
}

/**
 * Test Suite 3: BLE Service Tests
 */
ZTEST_SUITE(ble_service_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(ble_service_tests, test_ble_init)
{
    int ret = ble_imu_service_init();
    zassert_equal(ret, 0, "BLE service init should succeed");
}

ZTEST(ble_service_tests, test_ble_connection_status)
{
    /* Initially should be disconnected */
    bool connected = ble_imu_service_is_connected();
    zassert_false(connected, "Should be disconnected initially");
    
    bool subscribed = ble_imu_service_is_subscribed();
    zassert_false(subscribed, "Should not be subscribed initially");
}

ZTEST(ble_service_tests, test_ble_send_without_connection)
{
    attitude_t attitude = {
        .quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .euler = {0.0f, 0.0f, 0.0f},
        .timestamp = k_uptime_get_32()
    };
    
    int ret = ble_imu_service_send_attitude(&attitude);
    zassert_not_equal(ret, 0, "Send should fail when not connected");
}

/**
 * Test Suite 4: Integration Tests
 */
ZTEST_SUITE(integration_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(integration_tests, test_full_pipeline)
{
    /* Initialize all components */
    imu_config_t imu_config = {
        .sample_rate_hz = 100,
        .accel_range_g = 2,
        .gyro_range_dps = 250
    };
    
    int ret = imu_driver_init(&imu_config);
    zassert_equal(ret, 0, "IMU init failed");
    
    ret = attitude_fusion_init(0.1f, 0.01f);
    zassert_equal(ret, 0, "Attitude fusion init failed");
    
    /* Read IMU and process attitude */
    imu_data_t imu_data;
    attitude_t attitude;
    
    ret = imu_driver_read(&imu_data);
    zassert_equal(ret, 0, "IMU read failed");
    
    ret = attitude_fusion_update(&imu_data, &attitude);
    zassert_equal(ret, 0, "Attitude update failed");
    
    /* Verify output ranges */
    zassert_true(fabsf(attitude.euler.roll) <= M_PI,
                 "Roll angle out of valid range");
    zassert_true(fabsf(attitude.euler.pitch) <= M_PI / 2.0f,
                 "Pitch angle out of valid range");
    zassert_true(fabsf(attitude.euler.yaw) <= M_PI,
                 "Yaw angle out of valid range");
}

ZTEST(integration_tests, test_continuous_updates)
{
    const int num_samples = 10;
    imu_data_t imu_data;
    attitude_t attitude;
    
    for (int i = 0; i < num_samples; i++) {
        int ret = imu_driver_read(&imu_data);
        zassert_equal(ret, 0, "IMU read failed at sample %d", i);
        
        ret = attitude_fusion_update(&imu_data, &attitude);
        zassert_equal(ret, 0, "Attitude update failed at sample %d", i);
        
        /* Small delay between samples */
        k_msleep(10);
    }
}