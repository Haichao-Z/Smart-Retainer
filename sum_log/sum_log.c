#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "sum_log.h"


LOG_MODULE_REGISTER(sum_log, LOG_LEVEL_DBG);

int sum_log(int a, int b)
{
    int result = a + b;
    
    LOG_INF("=== Sum using Logger ===");
    LOG_INF("Computing sum of two integers");
    
    LOG_DBG("Input a: %d", a);
    LOG_DBG("Input b: %d", b);
    
    LOG_INF("Result: %d + %d = %d", a, b, result);
    
    LOG_HEXDUMP_INF(&a, sizeof(a), "Hexdump of input a:");
    LOG_HEXDUMP_INF(&b, sizeof(b), "Hexdump of input b:");
    LOG_HEXDUMP_INF(&result, sizeof(result), "Hexdump of result:");
    
    LOG_INF("========================");
    
    return result;
}