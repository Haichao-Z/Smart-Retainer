/**
 * @file main.c
 * @brief Smart Retainer with BNO055 IMU
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "bno055_driver.h"
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

/* Thread stack size */
#define IMU_THREAD_STACK_SIZE   4096
#define IMU_THREAD_PRIORITY     5

/* Thread stack */
K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

/* LED blink state */
static bool led_state = false;

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
 * @brief Wait for BNO055 calibration
 */
static void wait_for_calibration(void)
{
    bno055_calibration_t calib;
    bool calibrated = false;
    int led_counter = 0;

    LOG_INF("=== BNO055 CALIBRATION ===");
    LOG_INF("Move device in figure-8 pattern for magnetometer");
    LOG_INF("Rotate around all axes for gyroscope");
    LOG_INF("Place on stable surface for accelerometer");

    while (!calibrated) {
        if (bno055_get_calibration(&calib) == 0) {
            LOG_INF("Calibration: Sys=%d Gyro=%d Accel=%d Mag=%d",
                    calib.sys, calib.gyro, calib.accel, calib.mag);

            /* Check if sufficiently calibrated (at least 2 for each) */
            if (calib.sys >= 2 && calib.gyro >= 2 && 
                calib.accel >= 2 && calib.mag >= 2) {
                calibrated = true;
                LOG_INF("Calibration sufficient!");
            }
        }

        /* Blink LED during calibration */
        led_counter++;
        if (led_counter >= 5) {
            led_heartbeat();
            led_counter = 0;
        }

        k_msleep(200);
    }

    /* Flash LED to indicate calibration complete */
    for (int i = 0; i < 6; i++) {
        led_heartbeat();
        k_msleep(100);
    }
}

/**
 * @brief Convert BNO055 data to attitude structure
 */
static void bno055_to_attitude(const bno055_data_t *bno_data, attitude_t *attitude)
{
    /* Copy quaternion */
    attitude->quaternion.w = bno_data->quaternion.w;
    attitude->quaternion.x = bno_data->quaternion.x;
    attitude->quaternion.y = bno_data->quaternion.y;
    attitude->quaternion.z = bno_data->quaternion.z;

    /* Convert Euler angles to radians if needed and map correctly
     * BNO055: heading, roll, pitch
     * Our system: roll, pitch, yaw */
    attitude->euler.roll = bno_data->euler.roll * M_PI / 180.0f;
    attitude->euler.pitch = bno_data->euler.pitch * M_PI / 180.0f;
    attitude->euler.yaw = bno_data->euler.heading * M_PI / 180.0f;

    attitude->timestamp = bno_data->timestamp;
}

/**
 * @brief IMU processing thread
 */
static void imu_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    bno055_data_t bno_data;
    attitude_t attitude;
    int err;
    uint32_t led_counter = 0;

    LOG_INF("IMU thread started");

    /* Wait for initial calibration */
    k_msleep(1000);  /* Give sensor time to stabilize */

    while (1) {
        /* Read all BNO055 data */
        err = bno055_read_all(&bno_data);
        if (err) {
            LOG_ERR("Failed to read BNO055: %d", err);
            k_msleep(IMU_SAMPLE_PERIOD_MS);
            continue;
        }

        /* Convert to attitude structure */
        bno055_to_attitude(&bno_data, &attitude);

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
            
            /* Log current orientation and calibration */
            LOG_INF("Orientation: R=%.1f° P=%.1f° Y=%.1f° | Q[%.3f,%.3f,%.3f,%.3f]",
                    roll_deg, pitch_deg, yaw_deg,
                    (double)attitude.quaternion.w,
                    (double)attitude.quaternion.x,
                    (double)attitude.quaternion.y,
                    (double)attitude.quaternion.z);

            LOG_INF("Calibration: Sys=%d Gyro=%d Accel=%d Mag=%d",
                    bno_data.calibration.sys, bno_data.calibration.gyro,
                    bno_data.calibration.accel, bno_data.calibration.mag);
        }

        /* Sleep until next sample */
        k_msleep(IMU_SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief Initialize system components
 */
static int system_init(void)
{
    int err;

    LOG_INF("=== Smart Retainer Initialization (BNO055) ===");

    /* Initialize LED */
    if (device_is_ready(led.port)) {
        err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_WRN("LED init failed: %d", err);
        }
    }
    LOG_INF("[1/5] LED initialized");

    /* Initialize BNO055 */
    bno055_config_t bno_config = {
        .address = BNO055_ADDRESS_A,
        .mode = BNO055_OPERATION_MODE_NDOF,  /* 9-DOF fusion mode */
        .use_external_crystal = false,
        .units_in_radians = false  /* Use degrees, we'll convert */
    };

    err = bno055_init(&bno_config);
    if (err) {
        LOG_ERR("BNO055 init failed: %d", err);
        return err;
    }
    LOG_INF("[2/5] BNO055 initialized in NDOF mode");

    /* Run self-test */
    err = bno055_self_test();
    if (err) {
        LOG_WRN("BNO055 self-test failed: %d", err);
    }
    LOG_INF("[3/5] BNO055 self-test complete");

    /* Wait for calibration */
    wait_for_calibration();
    LOG_INF("[4/5] BNO055 calibrated");

    /* Initialize BLE service */
    k_msleep(100);
    err = ble_imu_service_init();
    if (err) {
        LOG_ERR("BLE init failed: %d", err);
        LOG_WRN("Continuing without BLE...");
    } else {
        LOG_INF("[5/5] BLE service initialized");
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

    LOG_INF("=== Smart Retainer MVP v3.0 (BNO055) ===");
    LOG_INF("Build: " __DATE__ " " __TIME__);
    LOG_INF("Features: BNO055 9-DOF with built-in fusion");

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