# CMakeLists.txt for Smart Retainer MVP
# Target: nRF7002 DK with Zephyr RTOS

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr package
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Project definition
project(smart_retainer_mvp VERSION 1.0.0)

# Source files
target_sources(app PRIVATE
    src/main.c
    src/imu_driver.c
    src/attitude_fusion.c
    src/ble_imu_service.c
)

# Include directories
target_include_directories(app PRIVATE
    src/
)

# Compiler options
target_compile_options(app PRIVATE
    -O0                         # 禁用优化
    -g3                         # 最大调试信息
    -Wall                       # 所有警告
    -Wno-double-promotion       # 忽略浮点提升警告
    -fno-omit-frame-pointer     # 保留帧指针（调试用）
    -ffunction-sections         # 每个函数独立段
    -fdata-sections             # 每个数据独立段
)

# Linker options
target_link_options(app PRIVATE
    -Wl,--gc-sections           # 垃圾回收未使用段
    -Wl,--print-memory-usage    # 打印内存使用
)

# 确保链接数学库（虽然Zephyr通常会自动处理）
target_link_libraries(app PRIVATE m)