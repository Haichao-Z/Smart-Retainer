#include <zephyr/ztest.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define LED_ALIAS DT_ALIAS(led5180)
#define BTN_ALIAS DT_ALIAS(button5180)

ZTEST(led_switch, test_dt_aliases_exist)
{
    zassert_true(DT_NODE_HAS_STATUS(LED_ALIAS, okay), 
                 "LED alias missing/disabled");
    zassert_true(DT_NODE_HAS_STATUS(BTN_ALIAS, okay), 
                 "Button alias missing/disabled");
}

ZTEST(led_switch, test_gpio_devices_ready)
{
    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_ALIAS, gpios);
    const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(BTN_ALIAS, gpios);
    
    zassert_true(gpio_is_ready_dt(&led), "LED GPIO not ready");
    zassert_true(gpio_is_ready_dt(&btn), "Button GPIO not ready");
}

ZTEST(led_switch, test_led_config)
{
    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_ALIAS, gpios);
    int ret;
    
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    zassert_equal(ret, 0, "Failed to configure LED");
}

ZTEST(led_switch, test_button_config)
{
    const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(BTN_ALIAS, gpios);
    int ret;
    
    ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
    zassert_equal(ret, 0, "Failed to configure button");
}

ZTEST(led_switch, test_led_toggle)
{
    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_ALIAS, gpios);
    int ret;
    
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    zassert_equal(ret, 0, "LED config failed");
    
    ret = gpio_pin_set_dt(&led, 1);
    zassert_equal(ret, 0, "Failed to set LED high");
    
    ret = gpio_pin_set_dt(&led, 0);
    zassert_equal(ret, 0, "Failed to set LED low");
}

ZTEST_SUITE(led_switch, NULL, NULL, NULL, NULL, NULL);