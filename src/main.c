/**
 * @file main.c
 * @brief Smart Retainer with Enhanced BNO055 IMU - Drift Prevention Version
 * 
 * Key improvements:
 * - Full sensor calibration before operation
 * - Averaged zero-point calibration
 * - External crystal oscillator support
 * - Calibration persistence
 * - Better drift detection and compensation
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

/* Zero-point calibration settings */
#define ZERO_CALIB_SAMPLES      20    /* Number of samples to average */
#define ZERO_CALIB_DELAY_MS     50    /* Delay between samples */

/* Thread stack */
#define IMU_THREAD_STACK_SIZE   4096
#define IMU_THREAD_PRIORITY     5

K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

/* LED blink state */
static bool led_state = false;

/* Flags */
volatile bool calibrate_zero_point = false;
static volatile bool save_calibration = false;

/* Statistics tracking */
static struct {
    uint32_t total_samples;
    uint32_t valid_samples;
    uint32_t invalid_samples;
    float max_norm_deviation;
    uint32_t last_calib_save_time;
} system_stats = {0};

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
        
    case BLE_IMU_CMD_CALIBRATE:
        LOG_INF("Sensor calibration requested via BLE");
        save_calibration = true;
        break;
        
    default:
        LOG_DBG("Unhandled BLE command: 0x%02x", cmd);
        break;
    }
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
 * @brief Button press handler - triggers zero-point calibration
 */
static void button_pressed(const struct device *dev, struct gpio_callback *cb, 
                          uint32_t pins)
{
    static uint32_t last_press = 0;
    uint32_t current = k_uptime_get_32();
    
    /* Debounce: ignore presses within 200ms */
    if (current - last_press < 200) {
        return;
    }
    last_press = current;
    
    /* Check for long press (>2 seconds) for calibration save */
    if (gpio_pin_get_dt(&button) == 1) {
        k_msleep(2000);
        if (gpio_pin_get_dt(&button) == 1) {
            LOG_INF("💾 Long press detected - saving calibration");
            save_calibration = true;
            led_flash_pattern(5, 100);
            return;
        }
    }
    
    /* Short press for zero-point calibration */
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
 * @brief Display calibration status with guidance
 */
static void display_calibration_status(const bno055_calibration_t *calib)
{
    LOG_INF("Calibration Status:");
    LOG_INF("  System:  [%s%s%s] %d/3", 
            calib->sys >= 1 ? "■" : "□",
            calib->sys >= 2 ? "■" : "□",
            calib->sys >= 3 ? "■" : "□",
            calib->sys);
    LOG_INF("  Gyro:    [%s%s%s] %d/3", 
            calib->gyro >= 1 ? "■" : "□",
            calib->gyro >= 2 ? "■" : "□",
            calib->gyro >= 3 ? "■" : "□",
            calib->gyro);
    LOG_INF("  Accel:   [%s%s%s] %d/3", 
            calib->accel >= 1 ? "■" : "□",
            calib->accel >= 2 ? "■" : "□",
            calib->accel >= 3 ? "■" : "□",
            calib->accel);
    LOG_INF("  Mag:     [%s%s%s] %d/3", 
            calib->mag >= 1 ? "■" : "□",
            calib->mag >= 2 ? "■" : "□",
            calib->mag >= 3 ? "■" : "□",
            calib->mag);
    
    /* Provide specific guidance for uncalibrated sensors */
    if (calib->gyro < 3) {
        LOG_INF("  → Gyro: Keep device completely still");
    }
    if (calib->accel < 3) {
        LOG_INF("  → Accel: Place in 6 different orientations");
    }
    if (calib->mag < 3) {
        LOG_INF("  → Mag: Move in figure-8 pattern away from metal");
    }
}

/**
 * @brief Convert BNO055 data to attitude structure with offset correction
 */
static void bno055_to_attitude(const bno055_data_t *bno_data, attitude_t *attitude)
{
    bno055_quaternion_t corrected_quat;
    
     /* 检查输入数据有效性 */
    if (!bno_data || !attitude) {
        return;
    }

    /* Apply orientation offset correction */
    orientation_offset_apply(&bno_data->quaternion, &corrected_quat);
    
    /* Validate quaternion */
    float norm = sqrtf(corrected_quat.w * corrected_quat.w + 
                      corrected_quat.x * corrected_quat.x +
                      corrected_quat.y * corrected_quat.y + 
                      corrected_quat.z * corrected_quat.z);
    
    if (fabsf(norm - 1.0f) > 0.01f) {
        system_stats.invalid_samples++;
        if (fabsf(norm - 1.0f) > system_stats.max_norm_deviation) {
            system_stats.max_norm_deviation = fabsf(norm - 1.0f);
        }
        LOG_DBG("Quaternion norm deviation: %.6f", fabsf(norm - 1.0f));
    } else {
        system_stats.valid_samples++;
    }
    
    /* Copy corrected quaternion */
    attitude->quaternion.w = corrected_quat.w;
    attitude->quaternion.x = corrected_quat.x;
    attitude->quaternion.y = corrected_quat.y;
    attitude->quaternion.z = corrected_quat.z;

    /* Convert Euler angles to radians if needed */
    if (bno_data->euler.heading <= 360.0f) {  /* Degrees */
        attitude->euler.roll = bno_data->euler.roll * M_PI / 180.0f;
        attitude->euler.pitch = bno_data->euler.pitch * M_PI / 180.0f;
        attitude->euler.yaw = bno_data->euler.heading * M_PI / 180.0f;
    } else {  /* Already in radians */
        attitude->euler.roll = bno_data->euler.roll;
        attitude->euler.pitch = bno_data->euler.pitch;
        attitude->euler.yaw = bno_data->euler.heading;
    }

    attitude->timestamp = bno_data->timestamp;
}

/**
 * @brief Perform averaged zero-point calibration
 */
static void perform_zero_point_calibration(void)
{
    bno055_quaternion_t samples[ZERO_CALIB_SAMPLES];
    bno055_data_t bno_data;
    int err;
    int valid_samples = 0;
    
    LOG_INF("");
    LOG_INF("╔══════════════════════════════════════╗");
    LOG_INF("║    🎯 ZERO-POINT CALIBRATION         ║");
    LOG_INF("╠══════════════════════════════════════╣");
    LOG_INF("║ 📋 Instructions:                     ║");
    LOG_INF("║   1. Wear the retainer comfortably   ║");
    LOG_INF("║   2. Look straight ahead at screen   ║");
    LOG_INF("║   3. Keep your head still            ║");
    LOG_INF("║                                      ║");
    LOG_INF("║ ⏳ Sampling %d points...             ║", ZERO_CALIB_SAMPLES);
    LOG_INF("╚══════════════════════════════════════╝");
    
    /* Flash LED to indicate calibration starting */
    led_flash_pattern(3, 300);
    
    /* Collect samples */
    for (int i = 0; i < ZERO_CALIB_SAMPLES; i++) {
        err = bno055_read_all(&bno_data);
        if (err) {
            LOG_WRN("Failed to read sample %d: %d", i, err);
            continue;
        }
        
        /* Check calibration status */
        if (bno_data.calibration.sys == 0) {
            LOG_WRN("System uncalibrated, sample may be unreliable");
        }
        
        samples[valid_samples] = bno_data.quaternion;
        valid_samples++;
        
        /* Show progress */
        if ((i + 1) % 5 == 0) {
            LOG_INF("  Progress: %d/%d samples", i + 1, ZERO_CALIB_SAMPLES);
        }
        
        k_msleep(ZERO_CALIB_DELAY_MS);
    }
    
    if (valid_samples < 5) {
        LOG_ERR("❌ Insufficient valid samples (%d/%d)", 
                valid_samples, ZERO_CALIB_SAMPLES);
        led_flash_pattern(10, 100);  /* Error pattern */
        return;
    }
    
    /* Set averaged zero point */
    err = orientation_offset_set_zero_averaged(samples, valid_samples);
    if (err) {
        LOG_ERR("❌ Failed to set zero point: %d", err);
        led_flash_pattern(10, 100);  /* Error pattern */
        return;
    }
    
    /* Get and display offset statistics */
    orientation_stats_t stats;
    orientation_offset_get_stats(&stats);
    
    LOG_INF("");
    LOG_INF("✅ Zero-point calibration complete!");
    LOG_INF("   Samples used: %d", valid_samples);
    LOG_INF("   Offset quaternion: (%.4f, %.4f, %.4f, %.4f)",
            (double)stats.offset_w, (double)stats.offset_x,
            (double)stats.offset_y, (double)stats.offset_z);
    LOG_INF("   Current orientation is now the reference");
    LOG_INF("╚══════════════════════════════════════╝");
    LOG_INF("");
    
    /* Success pattern: rapid flash */
    led_flash_pattern(10, 50);
}

/**
 * @brief Save sensor calibration
 */
static void save_sensor_calibration(void)
{
    int err;
    
    if (!bno055_is_fully_calibrated()) {
        LOG_WRN("Cannot save - sensor not fully calibrated");
        display_calibration_status(&(bno055_calibration_t){0});
        return;
    }
    
    err = bno055_save_calibration_profile();
    if (err) {
        LOG_ERR("Failed to save calibration: %d", err);
        return;
    }
    
    system_stats.last_calib_save_time = k_uptime_get_32();
    LOG_INF("💾 Sensor calibration saved successfully");
    led_flash_pattern(3, 200);
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
    uint32_t stats_counter = 0;

    LOG_INF("IMU thread started");

    /* Wait for initial stabilization */
    k_msleep(1000);

    while (1) {
        /* Check if zero-point calibration requested */
        if (calibrate_zero_point) {
            calibrate_zero_point = false;
            perform_zero_point_calibration();
        }
        
        /* Check if calibration save requested */
        if (save_calibration) {
            save_calibration = false;
            save_sensor_calibration();
        }

        /* Read all BNO055 data */
        err = bno055_read_all(&bno_data);
        if (err) {
            LOG_ERR("Failed to read BNO055: %d", err);
            k_msleep(IMU_SAMPLE_PERIOD_MS);
            continue;
        }
        
        system_stats.total_samples++;

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

        /* LED heartbeat and logging at 1 Hz */
        led_counter++;
        if (led_counter >= IMU_SAMPLE_RATE_HZ) {
            led_heartbeat();
            led_counter = 0;
            
            /* Log current orientation */
            LOG_INF("Orientation: R=%.1f° P=%.1f° Y=%.1f° | "
                    "Q[%.3f,%.3f,%.3f,%.3f] | Cal[%d%d%d%d] | %s",
                    (double)roll_deg, (double)pitch_deg, (double)yaw_deg,
                    (double)attitude.quaternion.w,
                    (double)attitude.quaternion.x,
                    (double)attitude.quaternion.y,
                    (double)attitude.quaternion.z,
                    bno_data.calibration.sys, bno_data.calibration.gyro,
                    bno_data.calibration.accel, bno_data.calibration.mag,
                    orientation_offset_is_set() ? "ZEROED" : "RELATIVE");
        }
        
        /* Display statistics every 30 seconds */
        stats_counter++;
        if (stats_counter >= IMU_SAMPLE_RATE_HZ * 30) {
            stats_counter = 0;
            
            orientation_stats_t offset_stats;
            orientation_offset_get_stats(&offset_stats);
            
            LOG_INF("=== System Statistics ===");
            LOG_INF("  Total samples: %lu", system_stats.total_samples);
            LOG_INF("  Valid samples: %lu (%.1f%%)", 
                    system_stats.valid_samples,
                    (double)(system_stats.valid_samples * 100.0f / 
                            system_stats.total_samples));
            LOG_INF("  Max norm deviation: %.6f", 
                    (double)system_stats.max_norm_deviation);
            LOG_INF("  Drift corrections: %lu", 
                    offset_stats.drift_corrections);
            
            if (!bno055_is_fully_calibrated()) {
                LOG_WRN("Sensor not fully calibrated!");
                display_calibration_status(&bno_data.calibration);
            }
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

    LOG_INF("=== Smart Retainer Initialization (Enhanced) ===");

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
                LOG_INF("[2/6] Button initialized");
                LOG_INF("  - Short press: Set zero point");
                LOG_INF("  - Long press (2s): Save calibration");
            }
        }
    }

    /* Initialize orientation offset system */
    orientation_offset_init();
    LOG_INF("[3/6] Orientation offset system initialized");

    /* Initialize BNO055 with enhanced configuration */
    bno055_config_t bno_config = {
        .address = BNO055_ADDRESS_A,
        .mode = BNO055_OPERATION_MODE_NDOF,
        .use_external_crystal = true,  /* Enable for better accuracy */
        .units_in_radians = false      /* Use degrees internally */
    };

    /* Enable auto-save of calibration */
    bno055_enable_auto_calibration_save(true);

    err = bno055_init(&bno_config);
    if (err) {
        LOG_ERR("BNO055 init failed: %d", err);
        return err;
    }
    LOG_INF("[4/6] BNO055 initialized in NDOF mode");

    /* Try to load saved calibration profile */
    err = bno055_load_calibration_profile();
    if (err == 0) {
        LOG_INF("[5/6] Loaded saved calibration profile");
    } else {
        LOG_INF("[5/6] No saved calibration, using auto-calibration");
    }

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
    LOG_INF("💡 USAGE INSTRUCTIONS:");
    LOG_INF("   • Wear the retainer comfortably");
    LOG_INF("   • Look straight at screen");
    LOG_INF("   • Press Button 1 (short) to set zero point");
    LOG_INF("   • Press Button 1 (long 2s) to save calibration");
    LOG_INF("   • Use BLE app for remote control");
    LOG_INF("");
    
    return 0;
}

/**
 * @brief Main function
 */
int main(void)
{
    int err;

    LOG_INF("╔════════════════════════════════════════╗");
    LOG_INF("║  Smart Retainer MVP v4.0 (Enhanced)   ║");
    LOG_INF("║  BNO055 + Drift Prevention            ║");
    LOG_INF("╠════════════════════════════════════════╣");
    LOG_INF("║  Build: " __DATE__ " " __TIME__ "      ║");
    LOG_INF("╚════════════════════════════════════════╝");

    err = system_init();
    if (err) {
        LOG_ERR("System initialization failed: %d", err);
        
        /* Flash error pattern */
        if (device_is_ready(led.port)) {
            while (1) {
                led_flash_pattern(3, 100);
                k_msleep(2000);
            }
        }
        return err;
    }

    /* Perform initial self-test */
    LOG_INF("Performing sensor self-test...");
    bno055_self_test();

    /* Create IMU processing thread */
    k_thread_create(&imu_thread_data, imu_thread_stack,
                    K_THREAD_STACK_SIZEOF(imu_thread_stack),
                    imu_thread,
                    NULL, NULL, NULL,
                    IMU_THREAD_PRIORITY, K_FP_REGS, K_NO_WAIT);

    k_thread_name_set(&imu_thread_data, "imu_thread");

    LOG_INF("✅ Smart Retainer started successfully");
    LOG_INF("🔄 System ready for operation");
    
    return 0;
}