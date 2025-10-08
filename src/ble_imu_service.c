/**
 * @file ble_imu_service.c
 * @brief BLE GATT service implementation for IMU data
 */

#include "ble_imu_service.h"
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/uuid.h>

LOG_MODULE_REGISTER(ble_imu_service, LOG_LEVEL_DBG);

/* Connection handle */
static struct bt_conn *current_conn = NULL;

/* Notification flags */
static bool quaternion_notify_enabled = false;
static bool euler_notify_enabled = false;

/* Forward declarations */
static void quaternion_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void euler_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t control_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

/* GATT Service Definition */
BT_GATT_SERVICE_DEFINE(imu_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_IMU_SERVICE),
    
    /* Quaternion Characteristic */
    BT_GATT_CHARACTERISTIC(BT_UUID_IMU_QUATERNION,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(quaternion_ccc_changed,
               BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    /* Euler Angles Characteristic */
    BT_GATT_CHARACTERISTIC(BT_UUID_IMU_EULER,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(euler_ccc_changed,
               BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    /* Control Characteristic */
    BT_GATT_CHARACTERISTIC(BT_UUID_IMU_CONTROL,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, control_write, NULL),
);

/**
 * @brief Connection callback
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed: %d", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    LOG_INF("BLE Connected");
}

/**
 * @brief Disconnection callback
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE Disconnected: reason %d", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    quaternion_notify_enabled = false;
    euler_notify_enabled = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/**
 * @brief Quaternion CCC changed callback
 */
static void quaternion_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    quaternion_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Quaternion notifications %s", 
            quaternion_notify_enabled ? "enabled" : "disabled");
}

/**
 * @brief Euler angles CCC changed callback
 */
static void euler_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    euler_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Euler notifications %s", 
            euler_notify_enabled ? "enabled" : "disabled");
}

/**
 * @brief Control characteristic write callback
 */
static ssize_t control_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset != 0 || len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t cmd = *(uint8_t *)buf;
    
    switch (cmd) {
    case BLE_IMU_CMD_START:
        LOG_INF("Received START command");
        /* Start IMU streaming */
        break;
        
    case BLE_IMU_CMD_STOP:
        LOG_INF("Received STOP command");
        /* Stop IMU streaming */
        break;
        
    case BLE_IMU_CMD_RESET:
        LOG_INF("Received RESET command");
        attitude_fusion_reset();
        break;
        
    case BLE_IMU_CMD_CALIBRATE:
        LOG_INF("Received CALIBRATE command");
        /* Perform calibration */
        break;
        
    default:
        LOG_WRN("Unknown control command: 0x%02x", cmd);
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    return len;
}

/**
 * @brief Initialize BLE IMU service
 */
int ble_imu_service_init(void)
{
    int err;

    LOG_INF("Initializing BLE IMU service");

    /* Enable Bluetooth */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed: %d", err);
        return err;
    }

    LOG_INF("Bluetooth initialized");

    /* Start advertising */
    struct bt_le_adv_param adv_param = {
        .id = BT_ID_DEFAULT,
        .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_NAME,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };

    struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_IMU_SERVICE_VAL),
    };

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start: %d", err);
        return err;
    }

    LOG_INF("BLE advertising started");
    return 0;
}

/**
 * @brief Send attitude data via BLE
 */
int ble_imu_service_send_attitude(const attitude_t *attitude)
{
    int err;

    if (!current_conn) {
        return -ENOTCONN;
    }

    /* Send quaternion data if subscribed */
    if (quaternion_notify_enabled) {
        ble_quaternion_packet_t quat_packet = {
            .w = attitude->quaternion.w,
            .x = attitude->quaternion.x,
            .y = attitude->quaternion.y,
            .z = attitude->quaternion.z,
            .timestamp = attitude->timestamp
        };

        err = bt_gatt_notify(current_conn, &imu_svc.attrs[1],
                            &quat_packet, sizeof(quat_packet));
        if (err) {
            LOG_WRN("Quaternion notify failed: %d", err);
        }
    }

    /* Send Euler angles if subscribed */
    if (euler_notify_enabled) {
        ble_euler_packet_t euler_packet = {
            .roll = attitude->euler.roll,
            .pitch = attitude->euler.pitch,
            .yaw = attitude->euler.yaw,
            .timestamp = attitude->timestamp
        };

        err = bt_gatt_notify(current_conn, &imu_svc.attrs[4],
                            &euler_packet, sizeof(euler_packet));
        if (err) {
            LOG_WRN("Euler notify failed: %d", err);
        }
    }

    return 0;
}

/**
 * @brief Check if notifications are enabled
 */
bool ble_imu_service_is_subscribed(void)
{
    return (quaternion_notify_enabled || euler_notify_enabled);
}

/**
 * @brief Check connection status
 */
bool ble_imu_service_is_connected(void)
{
    return (current_conn != NULL);
}