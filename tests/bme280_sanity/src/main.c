#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

/* 获取 BME280 设备 */
#define BME280_NODE DT_NODELABEL(bme280)

/* 测试 1: 检查设备树中是否存在 BME280 节点 */
ZTEST(bme280_sanity, test_devicetree_exists)
{
    zassert_true(DT_NODE_EXISTS(BME280_NODE), 
                 "BME280 node does not exist in devicetree");
    
    zassert_true(DT_NODE_HAS_STATUS(BME280_NODE, okay), 
                 "BME280 node is not enabled in devicetree");
}

/* 测试 2: 检查 BME280 设备是否就绪 */
ZTEST(bme280_sanity, test_device_ready)
{
    const struct device *bme280_dev = DEVICE_DT_GET(BME280_NODE);
    
    zassert_not_null(bme280_dev, 
                     "Failed to get BME280 device");
    
    zassert_true(device_is_ready(bme280_dev), 
                 "BME280 device is not ready");
}

/* 测试 3: 检查 I2C 总线是否就绪 */
ZTEST(bme280_sanity, test_i2c_ready)
{
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(arduino_i2c));
    
    zassert_not_null(i2c_dev, 
                     "Failed to get I2C device");
    
    zassert_true(device_is_ready(i2c_dev), 
                 "I2C bus is not ready");
}

/* 测试 4: 尝试从 BME280 读取传感器数据 */
ZTEST(bme280_sanity, test_sensor_fetch)
{
    const struct device *bme280_dev = DEVICE_DT_GET(BME280_NODE);
    struct sensor_value temp, press, humidity;
    int rc;
    
    /* 跳过如果设备未就绪 */
    zassert_true(device_is_ready(bme280_dev), 
                 "Device not ready");
    
    /* 尝试获取传感器数据 */
    rc = sensor_sample_fetch(bme280_dev);
    zassert_equal(rc, 0, 
                  "sensor_sample_fetch failed with error %d", rc);
    
    /* 尝试读取温度 */
    rc = sensor_channel_get(bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    zassert_equal(rc, 0, 
                  "Failed to get temperature channel");
    
    /* 尝试读取湿度 */
    rc = sensor_channel_get(bme280_dev, SENSOR_CHAN_HUMIDITY, &humidity);
    zassert_equal(rc, 0, 
                  "Failed to get humidity channel");
    
    /* 尝试读取气压 */
    rc = sensor_channel_get(bme280_dev, SENSOR_CHAN_PRESS, &press);
    zassert_equal(rc, 0, 
                  "Failed to get pressure channel");
    
    /* 打印读取的值（用于调试） */
    printk("Temperature: %d.%06d C\n", temp.val1, temp.val2);
    printk("Humidity: %d.%06d %%\n", humidity.val1, humidity.val2);
    printk("Pressure: %d.%06d kPa\n", press.val1, press.val2);
}

/* 注册测试套件 */
ZTEST_SUITE(bme280_sanity, NULL, NULL, NULL, NULL, NULL);