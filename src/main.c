/**
 * @file main.c
 * @brief Smart Retainer with Enhanced BNO055 IMU - IMU Mode (No Magnetometer)
 * 
 * Key improvements in this version:
 * - Uses IMU mode (accelerometer + gyroscope only) for faster calibration
 * - No magnetometer = no magnetic interference issues
 * - Automatic calibration persistence
 * - Dynamic calibration monitoring
 * - Better calibration guidance
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

/* Calibration check interval */
#define CALIB_CHECK_INTERVAL_MS 5000  /* Check every 5 seconds */

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
static volatile bool recalibrate_sensor = false;

/* Calibration state tracking */
static struct {
    uint32_t last_check_time;
    bool warning_shown;
    uint8_t last_sys_calib;
    uint8_t last_gyro_calib;
    uint8_t last_accel_calib;
} calib_state = {0};

/* Statistics tracking */
static struct {
    uint32_t total_samples;
    uint32_t valid_samples;
    uint32_t invalid_samples;
    float max_norm_deviation;
    uint32_t last_calib_save_time;
    uint32_t calib_warnings;
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
    LOG_INF("📊 Calibration Status:");
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
    
    /* IMU mode doesn't use magnetometer */
    LOG_INF("  Mag:     [---] (unused in IMU mode)");
    
    /* Provide specific guidance for uncalibrated sensors */
    if (calib->gyro < 3) {
        LOG_INF("  ⚠️  Gyro: Keep device COMPLETELY STILL for 3 seconds");
    }
    if (calib->accel < 3) {
        LOG_INF("  ⚠️  Accel: Place device in 6 orientations (±X, ±Y, ±Z)");
        LOG_INF("      Hold each position for 2-3 seconds");
    }
    if (calib->sys < 2) {
        LOG_INF("  ⚠️  System calibration insufficient");
    }
}

/**
 * @brief Monitor calibration status and alert if degraded
 */
static void monitor_calibration(const bno055_calibration_t *calib)
{
    uint32_t now = k_uptime_get_32();
    
    /* Check every CALIB_CHECK_INTERVAL_MS */
    if (now - calib_state.last_check_time < CALIB_CHECK_INTERVAL_MS) {
        return;
    }
    
    calib_state.last_check_time = now;
    
    /* Detect calibration degradation */
    bool calib_degraded = false;
    
    if (calib->sys < 2 || calib->gyro < 2 || calib->accel < 2) {
        calib_degraded = true;
    }
    
    /* Alert if calibration degraded and we haven't warned recently */
    if (calib_degraded && !calib_state.warning_shown) {
        system_stats.calib_warnings++;
        
        LOG_WRN("⚠️  CALIBRATION DEGRADED!");
        display_calibration_status(calib);
        
        /* Visual feedback */
        led_flash_pattern(3, 100);
        
        calib_state.warning_shown = true;
        
        LOG_INF("💡 TIP: Move to open area and recalibrate:");
        LOG_INF("   1. Keep device still (Gyro)");
        LOG_INF("   2. Place in 6 orientations (Accel)");
        
    } else if (!calib_degraded && calib_state.warning_shown) {
        /* Calibration recovered */
        LOG_INF("✅ Calibration recovered!");
        calib_state.warning_shown = false;
        
        /* Auto-save good calibration */
        if (calib->sys >= 3 && calib->gyro >= 3 && calib->accel >= 3) {
            save_calibration = true;
        }
    }
    
    /* Track calibration changes */
    if (calib->sys != calib_state.last_sys_calib ||
        calib->gyro != calib_state.last_gyro_calib ||
        calib->accel != calib_state.last_accel_calib) {
        
        LOG_DBG("Calibration update: S=%d G=%d A=%d",
                calib->sys, calib->gyro, calib->accel);
        
        calib_state.last_sys_calib = calib->sys;
        calib_state.last_gyro_calib = calib->gyro;
        calib_state.last_accel_calib = calib->accel;
    }
}

/**
 * @brief Convert BNO055 data to attitude structure with offset correction
 */
static void bno055_to_attitude(const bno055_data_t *bno_data, attitude_t *attitude)
{
    bno055_quaternion_t corrected_quat;
    
    /* Check input validity */
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
    LOG_INF("║   3. Keep your head STILL            ║");
    LOG_INF("║   4. Collecting %2d samples...        ║", ZERO_CALIB_SAMPLES);
    LOG_INF("╚══════════════════════════════════════╝");
    LOG_INF("");
    
    /* Visual feedback */
    led_flash_pattern(2, 100);
    k_msleep(1000);
    
    /* Collect samples */
    for (int i = 0; i < ZERO_CALIB_SAMPLES; i++) {
        err = bno055_read_all(&bno_data);
        if (err) {
            LOG_ERR("Failed to read sample %d: %d", i + 1, err);
            continue;
        }
        
        /* Validate quaternion */
        float norm = sqrtf(bno_data.quaternion.w * bno_data.quaternion.w +
                          bno_data.quaternion.x * bno_data.quaternion.x +
                          bno_data.quaternion.y * bno_data.quaternion.y +
                          bno_data.quaternion.z * bno_data.quaternion.z);
        
        if (fabsf(norm - 1.0f) < 0.05f) {  /* Accept if reasonably normalized */
            samples[valid_samples++] = bno_data.quaternion;
            
            if (i % 5 == 0) {
                LOG_INF("Progress: [%s%s%s%s] %d/%d",
                        i >= 5 ? "■" : "□",
                        i >= 10 ? "■" : "□",
                        i >= 15 ? "■" : "□",
                        i >= 20 ? "■" : "□",
                        i + 1, ZERO_CALIB_SAMPLES);
            }
        } else {
            LOG_WRN("Sample %d rejected (norm=%.4f)", i + 1, norm);
        }
        
        k_msleep(ZERO_CALIB_DELAY_MS);
    }
    
    if (valid_samples < ZERO_CALIB_SAMPLES / 2) {
        LOG_ERR("Insufficient valid samples (%d/%d)", valid_samples, ZERO_CALIB_SAMPLES);
        LOG_ERR("❌ Zero-point calibration FAILED");
        led_flash_pattern(5, 50);
        return;
    }
    
    /* Apply averaged offset */
    err = orientation_offset_set_zero_averaged(samples, valid_samples);
    if (err) {
        LOG_ERR("Failed to set zero offset: %d", err);
        led_flash_pattern(5, 50);
        return;
    }
    
    LOG_INF("");
    LOG_INF("✅ ZERO-POINT SET SUCCESSFULLY!");
    LOG_INF("   Valid samples: %d/%d", valid_samples, ZERO_CALIB_SAMPLES);
    LOG_INF("   Current orientation is now ZERO reference");
    LOG_INF("");
    
    /* Save to persistent storage */
    err = orientation_offset_save();
    if (err) {
        LOG_WRN("Failed to save offset to storage: %d", err);
    } else {
        LOG_INF("💾 Zero-point saved to persistent storage");
    }
    
    /* Success feedback */
    led_flash_pattern(3, 200);
}

/**
 * @brief Save sensor calibration profile
 */
static void save_sensor_calibration(void)
{
    int err;
    
    if (!bno055_is_fully_calibrated()) {
        bno055_calibration_t calib;
        bno055_get_calibration(&calib);
        
        LOG_WRN("Cannot save - sensor not fully calibrated");
        display_calibration_status(&calib);
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

    LOG_INF("🔄 IMU thread started");

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

        /* Monitor calibration status */
        monitor_calibration(&bno_data.calibration);

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
            LOG_INF("📐 R=%.1f° P=%.1f° Y=%.1f° | "
                    "Q[%.3f,%.3f,%.3f,%.3f] | Cal[S%dG%dA%d] | %s",
                    (double)roll_deg, (double)pitch_deg, (double)yaw_deg,
                    (double)attitude.quaternion.w,
                    (double)attitude.quaternion.x,
                    (double)attitude.quaternion.y,
                    (double)attitude.quaternion.z,
                    bno_data.calibration.sys, bno_data.calibration.gyro,
                    bno_data.calibration.accel,
                    orientation_offset_is_set() ? "ZEROED" : "RELATIVE");
        }
        
        /* Display statistics every 30 seconds */
        stats_counter++;
        if (stats_counter >= IMU_SAMPLE_RATE_HZ * 30) {
            stats_counter = 0;
            
            orientation_stats_t offset_stats;
            orientation_offset_get_stats(&offset_stats);
            
            LOG_INF("=== System Statistics ===");
            LOG_INF("  Uptime: %lu seconds", k_uptime_get_32() / 1000);
            LOG_INF("  Total samples: %lu", system_stats.total_samples);
            LOG_INF("  Valid samples: %lu (%.1f%%)", 
                    system_stats.valid_samples,
                    (double)(system_stats.valid_samples * 100.0f / 
                            system_stats.total_samples));
            LOG_INF("  Max norm deviation: %.6f", 
                    (double)system_stats.max_norm_deviation);
            LOG_INF("  Drift corrections: %lu", 
                    offset_stats.drift_corrections);
            LOG_INF("  Calibration warnings: %lu",
                    system_stats.calib_warnings);
            
            if (!bno055_is_fully_calibrated()) {
                LOG_WRN("⚠️  Sensor not fully calibrated!");
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

    LOG_INF("╔════════════════════════════════════════╗");
    LOG_INF("║  Smart Retainer Initialization         ║");
    LOG_INF("║  IMU Mode (Accel + Gyro Only)          ║");
    LOG_INF("╚════════════════════════════════════════╝");

    /* Initialize LED */
    if (device_is_ready(led.port)) {
        err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_WRN("LED init failed: %d", err);
        }
    }
    LOG_INF("[1/6] ✅ LED initialized");

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
                LOG_INF("[2/6] ✅ Button initialized");
                LOG_INF("       • Short press: Set zero point");
                LOG_INF("       • Long press (2s): Save calibration");
            }
        }
    }

    /* Initialize orientation offset system */
    orientation_offset_init();
    LOG_INF("[3/6] ✅ Orientation offset system initialized");

    /* Initialize BNO055 with IMU mode configuration */
    bno055_config_t bno_config = {
        .address = BNO055_ADDRESS_A,
        .mode = BNO055_OPERATION_MODE_IMU,  /* ⭐ IMU mode - no magnetometer */
        .use_external_crystal = true,
        .units_in_radians = false
    };

    /* Enable auto-save of calibration */
    bno055_enable_auto_calibration_save(true);

    err = bno055_init(&bno_config);
    if (err) {
        LOG_ERR("❌ BNO055 init failed: %d", err);
        return err;
    }
    LOG_INF("[4/6] ✅ BNO055 initialized in IMU mode");
    LOG_INF("       • Using: Accelerometer + Gyroscope");
    LOG_INF("       • No magnetometer (no magnetic interference)");

    /* Try to load saved calibration profile */
    err = bno055_load_calibration_profile();
    if (err == 0) {
        LOG_INF("[5/6] ✅ Loaded saved calibration profile");
    } else {
        LOG_INF("[5/6] ⏳ No saved calibration, starting auto-calibration");
        LOG_INF("       Please follow calibration instructions:");
        LOG_INF("       1. Keep device STILL for 3 seconds (Gyro)");
        LOG_INF("       2. Place in 6 orientations, 2s each (Accel)");
        LOG_INF("          ±X (left/right), ±Y (front/back), ±Z (up/down)");
    }

    /* Try to load saved zero-point offset */
    err = orientation_offset_load();
    if (err == 0) {
        LOG_INF("       ✅ Loaded saved zero-point offset");
    } else {
        LOG_INF("       ℹ️  No saved zero-point (use button to set)");
    }

    /* Initialize BLE service */
    k_msleep(100);
    err = ble_imu_service_init();
    if (err) {
        LOG_ERR("❌ BLE init failed: %d", err);
        LOG_WRN("⚠️  Continuing without BLE...");
    } else {
        LOG_INF("[6/6] ✅ BLE service initialized");
        
        /* Register BLE control command callback */
        ble_imu_service_register_control_callback(ble_control_handler);
        LOG_INF("       • BLE control callback registered");
    }

    LOG_INF("");
    LOG_INF("╔════════════════════════════════════════╗");
    LOG_INF("║  ✅ INITIALIZATION COMPLETE             ║");
    LOG_INF("╠════════════════════════════════════════╣");
    LOG_INF("║  💡 USAGE INSTRUCTIONS:                ║");
    LOG_INF("║   • Wear retainer comfortably          ║");
    LOG_INF("║   • Look straight ahead                ║");
    LOG_INF("║   • Press button to set zero point     ║");
    LOG_INF("║   • Long press to save calibration     ║");
    LOG_INF("║   • Use BLE app for remote control     ║");
    LOG_INF("╚════════════════════════════════════════╝");
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
    LOG_INF("║  Smart Retainer MVP v5.0 (IMU Mode)   ║");
    LOG_INF("║  BNO055 + Fast Calibration            ║");
    LOG_INF("╠════════════════════════════════════════╣");
    LOG_INF("║  Build: " __DATE__ " " __TIME__ "      ║");
    LOG_INF("╚════════════════════════════════════════╝");

    err = system_init();
    if (err) {
        LOG_ERR("❌ System initialization failed: %d", err);
        
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
    LOG_INF("🔍 Performing sensor self-test...");
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
    LOG_INF("");
    
    return 0;
}