cmake_minimum_required(VERSION 3.16)

# ==================== 交叉编译工具链 ====================
set(DEFAULT_TOOLCHAIN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6")
if(NOT EXISTS "${DEFAULT_TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-g++")
    set(DEFAULT_TOOLCHAIN_PATH "/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6")
endif()
set(TOOLCHAIN_PATH "${DEFAULT_TOOLCHAIN_PATH}"
    CACHE PATH "Toolchain path")

include(cmake/toolchain.cmake)

# ==================== 项目配置 ====================
project(gpio_proj LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")

# ==================== 板级选择 ====================
set(BOARD "2K301" CACHE STRING "Board type: 2K301")
add_definitions(-DBOARD_${BOARD})

# ==================== 官方库路径 ====================
# 查找顺序：旧位置(仓库内副本) -> vendor 官方 SDK -> 父仓裁剪版 libraries
set(LQ_LIB "")
foreach(cand IN ITEMS
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/Loongson_2K300_301_LIB/libraries"
        "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/loongson_sdk_V2.1.0/libraries"
        "${CMAKE_CURRENT_SOURCE_DIR}/../libraries")
    if(EXISTS "${cand}/common/inc/lq_utils.hpp" AND EXISTS "${cand}/common/src/lq_base_class.cpp")
        set(LQ_LIB "${cand}")
        break()
    endif()
endforeach()
if(NOT LQ_LIB)
    message(FATAL_ERROR
        "未找到可用的 Loongson libraries！\n"
        "请确认以下任一目录存在且包含 common/inc 与 common/src：\n"
        "  gpio_proj/lib/Loongson_2K300_301_LIB/libraries\n"
        "  vendor/loongson_sdk_V2.1.0/libraries\n"
        "  libraries（父仓裁剪版，缺少 lq_base_class.cpp，不可单独用于 gpio_proj）")
endif()
message(STATUS "  LQ_LIB   : ${LQ_LIB}")

set(LIB_COMMON_INC ${LQ_LIB}/common/inc)
set(LIB_COMMON_SRC ${LQ_LIB}/common/src)
set(LIB_DRV_INC    ${LQ_LIB}/drv/inc)
set(LIB_DRV_SRC    ${LQ_LIB}/drv/src)
set(SRC_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(CONFIG_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/config)

# ==================== 头文件路径 ====================
include_directories(
    ${LIB_COMMON_INC}
    ${LIB_DRV_INC}
    ${SRC_DIR}
    ${INCLUDE_DIR}
    ${CONFIG_DIR}
)

# ==================== 官方公共库 ====================
add_library(common_lib STATIC
    ${LIB_COMMON_SRC}/lq_utils.cpp
    ${LIB_COMMON_SRC}/lq_i2c_dev.cpp
    ${LIB_COMMON_SRC}/lq_base_class.cpp
)
target_include_directories(common_lib PUBLIC ${LIB_COMMON_INC})

# ==================== 官方驱动库 ====================
add_library(drv_lib STATIC
    ${LIB_DRV_SRC}/lq_reg_gpio.cpp
    ${LIB_DRV_SRC}/lq_reg_pwm.cpp
    ${LIB_DRV_SRC}/lq_reg_atim_pwm.cpp
    ${LIB_DRV_SRC}/lq_reg_gtim.pwm.cpp
    ${LIB_DRV_SRC}/lq_reg_adc.cpp
    ${LIB_DRV_SRC}/lq_reg_spi.cpp
    ${LIB_DRV_SRC}/lq_reg_pwm_encoder.cpp
    ${LIB_DRV_SRC}/lq_map_addr.cpp
    ${LIB_DRV_SRC}/lq_signal_handle.cpp
)
target_include_directories(drv_lib PUBLIC ${LIB_DRV_INC})
target_link_libraries(drv_lib PUBLIC common_lib)

# ==================== GPIO 看门狗守护进程 ====================
add_executable(gpio_manager
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/watchdog.cpp
)
target_link_libraries(gpio_manager common_lib drv_lib pthread rt)

# ==================== 测试程序 ====================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test)

add_executable(test_client
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_client.cpp
    ${SRC_DIR}/client_lib.cpp
)
target_link_libraries(test_client pthread rt)

# 电机看门狗实测（需要 link 驱动库控制硬件）
add_executable(test_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_motor_watchdog.cpp
    ${SRC_DIR}/client_lib.cpp
)
target_link_libraries(test_motor common_lib drv_lib pthread rt)

# ==================== 自定义目标 ====================
add_custom_target(manager DEPENDS gpio_manager)
add_custom_target(test DEPENDS test_client test_motor)

# ==================== 打印配置 ====================
message(STATUS "========================================")
message(STATUS "  Project: ${PROJECT_NAME}")
message(STATUS "  Toolchain: ${TOOLCHAIN_PATH}")
message(STATUS "  Board: ${BOARD}")
message(STATUS "  Mode: Pure watchdog (no relay)")
message(STATUS "========================================")
