#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "sum_printk.h"

int sum_printk(int a, int b)
{
    int result = a + b;
    
    printk("=== Sum using printk() ===\n");
    printk("Input a: %d\n", a);
    printk("Input b: %d\n", b);
    printk("Result: %d + %d = %d\n", a, b, result);
    printk("==========================\n");
    
    return result;
}