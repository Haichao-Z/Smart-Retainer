#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

ZTEST(basic_qemu, test_assertions)
{
    zassert_true(true, "True should be true");
    zassert_false(false, "False should be false");
    zassert_equal(1, 1, "1 should equal 1");
    zassert_not_equal(1, 2, "1 should not equal 2");
}

ZTEST(basic_qemu, test_math)
{
    int a = 10;
    int b = 20;
    int sum = a + b;
    
    zassert_equal(sum, 30, "10 + 20 should equal 30");
}

ZTEST(basic_qemu, test_strings)
{
    const char *str1 = "hello";
    const char *str2 = "hello";
    
    zassert_mem_equal(str1, str2, 5, "Strings should match");
}

ZTEST(basic_qemu, test_kernel)
{
    k_timeout_t timeout = K_MSEC(100);
    
    zassert_true(K_TIMEOUT_EQ(timeout, K_MSEC(100)), 
                 "Timeout should be 100ms");
}

ZTEST(basic_qemu, test_arrays)
{
    int arr[] = {1, 2, 3, 4, 5};
    int sum = 0;
    
    for (int i = 0; i < 5; i++) {
        sum += arr[i];
    }
    
    zassert_equal(sum, 15, "Array sum should be 15");
}

ZTEST_SUITE(basic_qemu, NULL, NULL, NULL, NULL, NULL);