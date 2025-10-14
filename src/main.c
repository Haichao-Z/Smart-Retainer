/**
 * @file main.c
 * @brief Smart Retainer MVP - Improved with Gyro Calibration
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#include "imu_driver.h"
#include "attitude_fusion.h"
#include "ble_imu_service.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* LED configuration */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Timing configuration */
#define IMU_SAMPLE_RATE_HZ      100     /* 100 Hz sampling rate */
#define IMU_SAMPLE_PERIOD_MS    (1000 / IMU_SAMPLE_RATE_HZ)
#define IMU_SAMPLE_PERIOD_S     (1.0f / IMU_SAMPLE_RATE_HZ)

/* Improved Madgwick filter gain */
#define MADGWICK_BETA           0.3f    /* Increased from 0.1 for better correction */

/* Calibration settings */
#define CALIBRATION_SAMPLES     200     /* Number of samples for gyro calibration */

/* Thread stack size */
#define IMU_THREAD_STACK_SIZE   4096
#define IMU_THREAD_PRIORITY     5

/* Thread stack */
K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

/* LED blink state */
static bool led_state = false;

/* Calibration buffer */
static imu_data_t calibration_buffer[CALIBRATION_SAMPLES];

/**
 * @brief LED heartbeat function
 */
static void led_heartbeat(void)
{
    if (!device_is_ready(led.port)) {
        return;
    }

    led_state = !led_state;
    gpio_pin_set_dt(&led, led_state);
}

/**
 * @brief Perform gyroscope calibration
 */
static int perform_gyro_calibration(void)
{
    int err;
    
    LOG_INF("=== GYROSCOPE CALIBRATION ===");
    LOG_INF("Keep device STATIONARY for 2 seconds...");
    
    /* Blink LED during calibration */
    for (int i = 0; i < 4; i++) {
        led_heartbeat();
        k_msleep(250);
    }
    
    /* Collect calibration samples */
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        err = imu_driver_read(&calibration_buffer[i]);
        if (err) {
            LOG_ERR("Failed to read IMU during calibration: %d", err);
            return err;
        }
        k_msleep(10);  /* 100 Hz sampling */
    }
    
    /* Perform calibration */
    err = attitude_fusion_calibrate_gyro(calibration_buffer, CALIBRATION_SAMPLES);
    if (err) {
        LOG_ERR("Gyro calibration failed: %d", err);
        return err;
    }
    
    LOG_INF("Gyro calibration complete!");
    
    /* Flash LED to indicate success */
    for (int i = 0; i < 6; i++) {
        led_heartbeat();
        k_msleep(100);
    }
    
    return 0;
}

/**
 * @brief IMU processing thread
 */
static void imu_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    imu_data_t imu_data;
    attitude_t attitude;
    int err;
    uint32_t led_counter = 0;

    LOG_INF("IMU thread started");

    /* Wait for calibration to complete */
    while (!attitude_fusion_is_calibrated()) {
        k_msleep(100);
    }

    while (1) {
        /* Read IMU data */
        err = imu_driver_read(&imu_data);
        if (err) {
            LOG_ERR("Failed to read IMU: %d", err);
            k_msleep(IMU_SAMPLE_PERIOD_MS);
            continue;
        }

        /* Update attitude estimation */
        err = attitude_fusion_update(&imu_data, &attitude);
        if (err) {
            LOG_ERR("Failed to update attitude: %d", err);
            k_msleep(IMU_SAMPLE_PERIOD_MS);
            continue;
        }

        /* Convert radians to degrees for logging */
        float roll_deg = attitude.euler.roll * 180.0f / M_PI;
        float pitch_deg = attitude.euler.pitch * 180.0f / M_PI;
        float yaw_deg = attitude.euler.yaw * 180.0f / M_PI;

        LOG_DBG("Attitude: R=%.1f P=%.1f Y=%.1f", 
                roll_deg, pitch_deg, yaw_deg);

        /* Send data via BLE if connected */
        if (ble_imu_service_is_subscribed()) {
            err = ble_imu_service_send_attitude(&attitude);
            if (err && err != -ENOTCONN) {
                LOG_WRN("Failed to send attitude: %d", err);
            }
        }

        /* LED heartbeat at 1 Hz */
        led_counter++;
        if (led_counter >= IMU_SAMPLE_RATE_HZ) {
            led_heartbeat();
            led_counter = 0;
            
            /* Log current orientation */
            LOG_INF("Orientation: R=%.1f° P=%.1f° Y=%.1f° | Q[%.3f,%.3f,%.3f,%.3f]",
                    roll_deg, pitch_deg, yaw_deg,
                    (double)attitude.quaternion.w,
                    (double)attitude.quaternion.x,
                    (double)attitude.quaternion.y,
                    (double)attitude.quaternion.z);
        }

        /* Sleep until next sample */
        k_msleep(IMU_SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief Scan I2C bus for devices
 */
static void scan_i2c_bus(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return;
    }
    
    LOG_INF("Scanning I2C bus...");
    
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        struct i2c_msg msgs[1];
        uint8_t dummy_data = 0;
        
        msgs[0].buf = &dummy_data;
        msgs[0].len = 0;
        msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
        
        int ret = i2c_transfer(i2c_dev, msgs, 1, addr);
        
        if (ret == 0) {
            LOG_INF("Found device at address 0x%02X", addr);
        }
    }
    
    LOG_INF("I2C scan complete");
}

/**
 * @brief Initialize system components
 */
static int system_init(void)
{
    int err;

    LOG_INF("=== Smart Retainer Initialization ===");

    /* Scan I2C bus */
    scan_i2c_bus();
    LOG_INF("[1/7] I2C scan complete");

    /* Initialize LED */
    if (device_is_ready(led.port)) {
        err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_WRN("LED init failed: %d", err);
        }
    }
    LOG_INF("[2/7] LED initialized");

    /* Initialize IMU driver */
    imu_config_t imu_config = {
        .sample_rate_hz = IMU_SAMPLE_RATE_HZ,
        .accel_range_g = 2,
        .gyro_range_dps = 250
    };

    err = imu_driver_init(&imu_config);
    if (err) {
        LOG_ERR("IMU init failed: %d", err);
        return err;
    }
    LOG_INF("[3/7] IMU driver initialized");

    /* Run IMU self-test */
    err = imu_driver_self_test();
    if (err) {
        LOG_WRN("IMU self-test failed: %d", err);
    }
    LOG_INF("[4/7] IMU self-test complete");

    /* Initialize attitude fusion */
    err = attitude_fusion_init(MADGWICK_BETA, IMU_SAMPLE_PERIOD_S);
    if (err) {
        LOG_ERR("Attitude fusion init failed: %d", err);
        return err;
    }
    LOG_INF("[5/7] Attitude fusion initialized (beta=%.2f)", (double)MADGWICK_BETA);

    /* Perform gyro calibration */
    err = perform_gyro_calibration();
    if (err) {
        LOG_ERR("Gyro calibration failed: %d", err);
        return err;
    }
    LOG_INF("[6/7] Gyroscope calibrated");

    /* Initialize BLE service */
    k_msleep(100);
    err = ble_imu_service_init();
    if (err) {
        LOG_ERR("BLE init failed: %d", err);
        LOG_WRN("Continuing without BLE...");
    } else {
        LOG_INF("[7/7] BLE service initialized");
    }

    LOG_INF("=== Initialization Complete ===");
    return 0;
}

/**
 * @brief Main function
 */
int main(void)
{
    int err;

    LOG_INF("=== Smart Retainer MVP v2.0 ===");
    LOG_INF("Build: " __DATE__ " " __TIME__);
    LOG_INF("Features: Gyro calibration, drift reduction");

    /* Initialize system */
    err = system_init();
    if (err) {
        LOG_ERR("System initialization failed: %d", err);
        return err;
    }

    /* Create IMU processing thread */
    k_thread_create(&imu_thread_data, imu_thread_stack,
                    K_THREAD_STACK_SIZEOF(imu_thread_stack),
                    imu_thread,
                    NULL, NULL, NULL,
                    IMU_THREAD_PRIORITY, K_FP_REGS, K_NO_WAIT);

    k_thread_name_set(&imu_thread_data, "imu_thread");

    LOG_INF("Smart Retainer started successfully");
    LOG_INF("Waiting for BLE connection...");

    return 0;
}