# ==================== 龙芯2K300/301 交叉编译工具链配置 ====================

# 目标系统
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR loongarch64)

# 交叉编译器
set(CMAKE_C_COMPILER "${TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-g++")

# 查找程序不在目标系统上搜索
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# 库和头文件只在目标系统上搜索
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Sysroot (如果工具链包含)
# set(CMAKE_SYSROOT "${TOOLCHAIN_PATH}/loongarch64-linux-gnu/sysroot")
