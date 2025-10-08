// // #include <stdio.h>
// // #include <zephyr/kernel.h>
// // #include <zephyr/drivers/gpio.h>
// // #include <zephyr/device.h>
// // #include <zephyr/devicetree.h>
// // #include <zephyr/drivers/sensor.h>
// // #include <zephyr/drivers/i2c.h>
// // #include <zephyr/logging/log.h>

// // #ifdef CONFIG_SUM_PRINT
// // #include "sum_printk.h"
// // #elif CONFIG_SUM_LOG
// // #include "sum_log.h"
// // #endif

// // LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

// // #define LED5180_NODE DT_ALIAS(led5180)
// // #define Button5180_NODE DT_ALIAS(button5180)

// // static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED5180_NODE, gpios);
// // static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(Button5180_NODE, gpios);


// // int main(void)
// // {
// //     int ret;
// //     bool led_state = true;

// //     printk("Starting application...\n");

// // #ifdef CONFIG_SUM_PRINT
// //     int result = sum_printk(10, 25);
// // #elif CONFIG_SUM_LOG
// //     int result = sum_log(10, 25);
// // #endif

// //     printk("Final result: %d\n", result);

// //     if (!gpio_is_ready_dt(&led)) {
// //         printk("LED not ready\n");
// //         return 0;
// //     }

// //     ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
// //     if (ret < 0) {
// //         return 0;
// //     }
    
// //     ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
// //     if (ret < 0) {
// //         return 0;
// //     }

// //     int button_state = gpio_pin_get_dt(&button);
// //     int button_last_state = button_state;

// //     while (1) {
// //         button_state = gpio_pin_get_dt(&button);
// //         if (button_state && (button_state != button_last_state)) {
// //             k_msleep(100);
// //             button_state = gpio_pin_get_dt(&button);
// //             if (button_state) {
// //                 led_state = !led_state;
// //                 ret = gpio_pin_set_dt(&led, (int)led_state);
// //                 if (ret < 0) {
// //                     return 0;
// //                 }
// //                 printk("LED state: %s\n", led_state ? "ON" : "OFF");
// //             }
// //         }
// //         button_last_state = button_state;
// //         k_msleep(10);
// //     }
// //     return 0;
// // }



// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/sensor.h>
// #include <zephyr/drivers/i2c.h>
// #include <zephyr/logging/log.h>

// LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

// int main(void)
// {
//     printk("\n=== nRF7002 DK BME280 Test ===\n");
//     printk("I2C on P1.02 (SDA) and P1.03 (SCL)\n\n");
    
//     // 使用 i2c1
//     const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    
//     if (i2c_dev == NULL) {
//         printk("ERROR: I2C device is NULL!\n");
//         return -1;
//     }
    
//     if (!device_is_ready(i2c_dev)) {
//         printk("ERROR: I2C device not ready!\n");
//         return -1;
//     }
//     printk("✓ I2C1 device ready (P1.02/P1.03)\n");
    
//     printk("\nScanning I2C bus...\n");
//     uint8_t cnt = 0;
    
//     /* Safe I2C bus scan: write one dummy byte (0x00) */
// 	for (uint8_t addr = 0x08; addr < 0x78; addr++) {
// 		uint8_t dummy = 0x00;
// 		struct i2c_msg msg = {
// 			.buf = &dummy,
// 			.len = 1,
// 			.flags = I2C_MSG_WRITE | I2C_MSG_STOP
// 		};

// 		int ret = i2c_transfer(i2c_dev, &msg, 1, addr);
// 		if (ret == 0) {
// 			printk("  [0x%02x] Device found\n", addr);
// 			cnt++;
// 		}
// 	}

    
//     printk("Scan complete: %d device(s) found\n\n", cnt);
    
//     if (cnt == 0) {
//         printk("⚠ No I2C devices detected!\n");
//         printk("Check BME280 wiring:\n");
//         printk("  SDA -> P1.02\n");
//         printk("  SCL -> P1.03\n");
//         printk("  VCC -> 3.3V or 1.8V (check BME280 voltage)\n");
//         printk("  GND -> GND\n");
//         return -1;
//     }
    
//     const struct device *bme280_dev = DEVICE_DT_GET(DT_NODELABEL(bme280));
    
//     if (bme280_dev == NULL) {
//         printk("ERROR: BME280 device is NULL!\n");
//         return -1;
//     }
    
//     if (!device_is_ready(bme280_dev)) {
//         printk("ERROR: BME280 not ready!\n");
//         printk("If scan found device at different address, update overlay\n");
//         return -1;
//     }
//     printk("BME280 sensor ready\n\n");
    
//     struct sensor_value temp, press, humidity;
//     int read_count = 0;
    
//     printk("Starting sensor readings...\n\n");
    
//     while (1) {
//         int rc = sensor_sample_fetch(bme280_dev);
        
//         if (rc == 0) {
//             sensor_channel_get(bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
//             sensor_channel_get(bme280_dev, SENSOR_CHAN_PRESS, &press);
//             sensor_channel_get(bme280_dev, SENSOR_CHAN_HUMIDITY, &humidity);
            
//             printk("[%d] T: %d.%02d°C | P: %d.%03d kPa | H: %d.%02d%%\n",
//                    ++read_count,
//                    temp.val1, temp.val2/10000,
//                    press.val1, press.val2/1000,
//                    humidity.val1, humidity.val2/10000);
//         } else {
//             printk("Sensor read failed: %d\n", rc);
//         }
        
//         k_sleep(K_SECONDS(2));
//     }
    
//     return 0;
// }

/**
 * @file main.c
 * @brief Smart Retainer MVP - Main Application
 * 
 * Real-time head orientation tracking using LSM6DS0 IMU
 * Data streaming via BLE to 3D visualization client
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

/* Madgwick filter gain */
#define MADGWICK_BETA           0.1f

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
        float roll_deg = attitude.euler.roll * 180.0f / 3.14159f;
        float pitch_deg = attitude.euler.pitch * 180.0f / 3.14159f;
        float yaw_deg = attitude.euler.yaw * 180.0f / 3.14159f;

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
            LOG_INF("Head orientation: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°",
                    roll_deg, pitch_deg, yaw_deg);
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

    LOG_INF("Initializing system...");

    scan_i2c_bus();
    LOG_INF("[1/6] I2C scan done");

    /* Initialize LED */
    if (device_is_ready(led.port)) {
        err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_WRN("LED init failed: %d", err);
        }
    }
    LOG_INF("[2/6] LED done");

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
    LOG_INF("[3/6] IMU done");

    /* Initialize attitude fusion */
    err = attitude_fusion_init(MADGWICK_BETA, IMU_SAMPLE_PERIOD_S);
    if (err) {
        LOG_ERR("Attitude init failed: %d", err);
        return err;
    }
    LOG_INF("[4/6] Attitude done");

    /* Initialize BLE service */
    k_msleep(100);  // 给系统一点喘息时间
    err = ble_imu_service_init();
    if (err) {
        LOG_ERR("BLE init failed: %d", err);
        // 不要返回错误，继续运行
        LOG_WRN("Continuing without BLE...");
    } else {
        LOG_INF("[5/6] BLE done");
    }

    printk("[6/6] All init complete!\n");
    return 0;
}

/**
 * @brief Main function
 */
int main(void)
{
    int err;

    LOG_INF("=== Smart Retainer MVP ===");
    LOG_INF("Build time: " __DATE__ " " __TIME__);

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