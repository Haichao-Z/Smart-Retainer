/**
 * @file main.c
 * @brief Smart Retainer with BNO055 IMU and Zero-Point Calibration
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "bno055_driver.h"
#include "ble_imu_service.h"
#include "orientation_offset.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* LED configuration */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Button for zero-point calibration */
#define BUTTON_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static struct gpio_callback button_cb_data;

/* Timing configuration */
#define IMU_SAMPLE_RATE_HZ      100
#define IMU_SAMPLE_PERIOD_MS    (1000 / IMU_SAMPLE_RATE_HZ)

/* Thread stack */
#define IMU_THREAD_STACK_SIZE   4096
#define IMU_THREAD_PRIORITY     5

K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

/* LED blink state */
static bool led_state = false;

/* Flag for zero-point calibration request */
static volatile bool calibrate_zero_point = false;

/**
 * @brief BLE control command handler
 */
static void ble_control_handler(uint8_t cmd)
{
    switch (cmd) {
    case BLE_IMU_CMD_SET_ZERO:
        LOG_INF("🎯 Zero-point calibration requested via BLE");
        calibrate_zero_point = true;
        break;
        
    case BLE_IMU_CMD_RESET:
        LOG_INF("Reset requested via BLE");
        orientation_offset_reset();
        break;
        
    default:
        LOG_DBG("Unhandled BLE command: 0x%02x", cmd);
        break;
    }
}

/**
 * @brief Button press handler - triggers zero-point calibration
 */
static void button_pressed(const struct device *dev, struct gpio_callback *cb, 
                          uint32_t pins)
{
    calibrate_zero_point = true;
    LOG_INF("🎯 Zero-point calibration requested!");
}

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
 * @brief Flash LED pattern
 */
static void led_flash_pattern(int count, int delay_ms)
{
    for (int i = 0; i < count; i++) {
        gpio_pin_set_dt(&led, 1);
        k_msleep(delay_ms);
        gpio_pin_set_dt(&led, 0);
        k_msleep(delay_ms);
    }
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

            if (calib.sys >= 2 && calib.gyro >= 2 && 
                calib.accel >= 2 && calib.mag >= 2) {
                calibrated = true;
                LOG_INF("Calibration sufficient!");
            }
        }

        led_counter++;
        if (led_counter >= 5) {
            led_heartbeat();
            led_counter = 0;
        }

        k_msleep(200);
    }

    led_flash_pattern(6, 100);
}

/**
 * @brief Convert BNO055 data to attitude structure with offset correction
 */
static void bno055_to_attitude(const bno055_data_t *bno_data, attitude_t *attitude)
{
    bno055_quaternion_t corrected_quat;
    
    /* Apply orientation offset correction */
    orientation_offset_apply(&bno_data->quaternion, &corrected_quat);
    
    /* Copy corrected quaternion */
    attitude->quaternion.w = corrected_quat.w;
    attitude->quaternion.x = corrected_quat.x;
    attitude->quaternion.y = corrected_quat.y;
    attitude->quaternion.z = corrected_quat.z;

    /* Convert Euler angles to radians */
    attitude->euler.roll = bno_data->euler.roll * M_PI / 180.0f;
    attitude->euler.pitch = bno_data->euler.pitch * M_PI / 180.0f;
    attitude->euler.yaw = bno_data->euler.heading * M_PI / 180.0f;

    attitude->timestamp = bno_data->timestamp;
}

/**
 * @brief Perform zero-point calibration
 */
static void perform_zero_point_calibration(void)
{
    bno055_data_t bno_data;
    int err;
    
    LOG_INF("");
    LOG_INF("════════════════════════════════════════");
    LOG_INF("🎯 ZERO-POINT CALIBRATION");
    LOG_INF("════════════════════════════════════════");
    LOG_INF("📋 Instructions:");
    LOG_INF("   1. Wear the retainer comfortably");
    LOG_INF("   2. Look straight ahead at the screen");
    LOG_INF("   3. Keep your head still");
    LOG_INF("");
    LOG_INF("⏳ Calibrating in 3 seconds...");
    
    /* Flash LED to indicate calibration starting */
    led_flash_pattern(3, 300);
    k_msleep(1000);
    
    /* Read current orientation */
    err = bno055_read_all(&bno_data);
    if (err) {
        LOG_ERR("❌ Failed to read IMU data: %d", err);
        return;
    }
    
    /* Set current orientation as zero point */
    err = orientation_offset_set_zero(&bno_data.quaternion);
    if (err) {
        LOG_ERR("❌ Failed to set zero point: %d", err);
        return;
    }
    
    LOG_INF("");
    LOG_INF("✅ Zero-point calibration complete!");
    LOG_INF("   Current orientation is now the reference");
    LOG_INF("════════════════════════════════════════");
    LOG_INF("");
    
    /* Success pattern: rapid flash */
    led_flash_pattern(10, 50);
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
    k_msleep(1000);

    while (1) {
        /* Check if zero-point calibration requested */
        if (calibrate_zero_point) {
            calibrate_zero_point = false;
            perform_zero_point_calibration();
        }

        /* Read all BNO055 data */
        err = bno055_read_all(&bno_data);
        if (err) {
            LOG_ERR("Failed to read BNO055: %d", err);
            k_msleep(IMU_SAMPLE_PERIOD_MS);
            continue;
        }

        /* Convert to attitude structure (with offset correction) */
        bno055_to_attitude(&bno_data, &attitude);

        /* Convert radians to degrees for logging */
        float roll_deg = attitude.euler.roll * 180.0f / M_PI;
        float pitch_deg = attitude.euler.pitch * 180.0f / M_PI;
        float yaw_deg = attitude.euler.yaw * 180.0f / M_PI;

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
            LOG_INF("Orientation: R=%.1f° P=%.1f° Y=%.1f° | Q[%.3f,%.3f,%.3f,%.3f] | %s",
                    roll_deg, pitch_deg, yaw_deg,
                    (double)attitude.quaternion.w,
                    (double)attitude.quaternion.x,
                    (double)attitude.quaternion.y,
                    (double)attitude.quaternion.z,
                    orientation_offset_is_set() ? "CALIBRATED" : "UNCALIBRATED");

            LOG_INF("Calibration: Sys=%d Gyro=%d Accel=%d Mag=%d",
                    bno_data.calibration.sys, bno_data.calibration.gyro,
                    bno_data.calibration.accel, bno_data.calibration.mag);
        }

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
    LOG_INF("[1/6] LED initialized");

    /* Initialize Button */
    if (device_is_ready(button.port)) {
        err = gpio_pin_configure_dt(&button, GPIO_INPUT);
        if (err) {
            LOG_WRN("Button init failed: %d", err);
        } else {
            err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
            if (err) {
                LOG_WRN("Button interrupt config failed: %d", err);
            } else {
                gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
                gpio_add_callback(button.port, &button_cb_data);
                LOG_INF("[2/6] Button initialized (press to set zero point)");
            }
        }
    }

    /* Initialize orientation offset system */
    orientation_offset_init();
    LOG_INF("[3/6] Orientation offset system initialized");

    /* Initialize BNO055 */
    bno055_config_t bno_config = {
        .address = BNO055_ADDRESS_A,
        .mode = BNO055_OPERATION_MODE_NDOF,
        .use_external_crystal = false,
        .units_in_radians = false
    };

    err = bno055_init(&bno_config);
    if (err) {
        LOG_ERR("BNO055 init failed: %d", err);
        return err;
    }
    LOG_INF("[4/6] BNO055 initialized in NDOF mode");

    /* Wait for calibration */
    wait_for_calibration();
    LOG_INF("[5/6] BNO055 calibrated");

    /* Initialize BLE service */
    k_msleep(100);
    err = ble_imu_service_init();
    if (err) {
        LOG_ERR("BLE init failed: %d", err);
        LOG_WRN("Continuing without BLE...");
    } else {
        LOG_INF("[6/6] BLE service initialized");
        
        /* Register BLE control command callback */
        ble_imu_service_register_control_callback(ble_control_handler);
        LOG_INF("BLE control callback registered");
    }

    LOG_INF("=== Initialization Complete ===");
    LOG_INF("");
    LOG_INF("💡 USAGE:");
    LOG_INF("   • Wear the retainer");
    LOG_INF("   • Look straight at screen");
    LOG_INF("   • Press Button 1 to set zero point");
    LOG_INF("");
    
    return 0;
}

/**
 * @brief Main function
 */
int main(void)
{
    int err;

    LOG_INF("=== Smart Retainer MVP v3.1 (BNO055 + Zero-Point) ===");
    LOG_INF("Build: " __DATE__ " " __TIME__);

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
    
    return 0;
}