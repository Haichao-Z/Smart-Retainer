/**
 * @file ble_imu_service.h
 * @brief BLE GATT service for IMU data transmission
 * 
 * Custom GATT service for streaming attitude data via BLE
 * Compatible with BNO055 IMU driver
 */

#ifndef BLE_IMU_SERVICE_H
#define BLE_IMU_SERVICE_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Custom UUID for IMU Service: 12345678-1234-5678-1234-56789abcdef0 */
#define BT_UUID_IMU_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

/* Quaternion Characteristic UUID: 12345678-1234-5678-1234-56789abcdef1 */
#define BT_UUID_IMU_QUATERNION_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

/* Euler Angles Characteristic UUID: 12345678-1234-5678-1234-56789abcdef2 */
#define BT_UUID_IMU_EULER_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

/* Control Characteristic UUID: 12345678-1234-5678-1234-56789abcdef3 */
#define BT_UUID_IMU_CONTROL_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3)

#define BT_UUID_IMU_SERVICE      BT_UUID_DECLARE_128(BT_UUID_IMU_SERVICE_VAL)
#define BT_UUID_IMU_QUATERNION   BT_UUID_DECLARE_128(BT_UUID_IMU_QUATERNION_VAL)
#define BT_UUID_IMU_EULER        BT_UUID_DECLARE_128(BT_UUID_IMU_EULER_VAL)
#define BT_UUID_IMU_CONTROL      BT_UUID_DECLARE_128(BT_UUID_IMU_CONTROL_VAL)

/* Quaternion structure */
typedef struct {
    float w;  /* Scalar part */
    float x;  /* Vector part X */
    float y;  /* Vector part Y */
    float z;  /* Vector part Z */
} quaternion_t;

/* Euler angles structure (in radians) */
typedef struct {
    float roll;   /* Roll angle (rotation around X-axis) */
    float pitch;  /* Pitch angle (rotation around Y-axis) */
    float yaw;    /* Yaw angle (rotation around Z-axis) */
} euler_angles_t;

/* Attitude data structure */
typedef struct {
    quaternion_t quaternion;    /* Orientation as quaternion */
    euler_angles_t euler;       /* Orientation as Euler angles */
    uint32_t timestamp;         /* Timestamp in milliseconds */
} attitude_t;

/* Data packet structures for BLE transmission */
typedef struct __attribute__((packed)) {
    float w;
    float x;
    float y;
    float z;
    uint32_t timestamp;
} ble_quaternion_packet_t;

typedef struct __attribute__((packed)) {
    float roll;
    float pitch;
    float yaw;
    uint32_t timestamp;
} ble_euler_packet_t;

/* Control commands */
typedef enum {
    BLE_IMU_CMD_START = 0x01,
    BLE_IMU_CMD_STOP = 0x02,
    BLE_IMU_CMD_RESET = 0x03,
    BLE_IMU_CMD_CALIBRATE = 0x04,
    BLE_IMU_CMD_SET_ZERO = 0x05  /* NEW: Set zero-point calibration */
} ble_imu_control_cmd_t;

/**
 * @brief Control command callback type
 * 
 * Register a callback to handle control commands from BLE client
 */
typedef void (*ble_imu_control_callback_t)(uint8_t cmd);

/**
 * @brief Initialize BLE IMU service
 * 
 * @return 0 on success, negative errno on failure
 */
int ble_imu_service_init(void);

/**
 * @brief Send attitude data via BLE notifications
 * 
 * @param attitude Pointer to attitude data
 * @return 0 on success, negative errno on failure
 */
int ble_imu_service_send_attitude(const attitude_t *attitude);

/**
 * @brief Check if BLE client is connected and subscribed
 * 
 * @return true if notifications are enabled
 */
bool ble_imu_service_is_subscribed(void);

/**
 * @brief Get connection status
 * 
 * @return true if BLE is connected
 */
bool ble_imu_service_is_connected(void);

/**
 * @brief Register callback for control commands
 * 
 * @param callback Function to call when control command received
 */
void ble_imu_service_register_control_callback(ble_imu_control_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* BLE_IMU_SERVICE_H */