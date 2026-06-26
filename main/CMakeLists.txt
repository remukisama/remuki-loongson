cmake_minimum_required(VERSION 3.16)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/toolchain_path.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/toolchain_path.cmake")
endif()

set(CMAKE_C_COMPILER "${CMAKE_TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "${CMAKE_TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-g++")

project(main)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Safety logic uses std::isfinite and ordered floating-point comparisons.
# Do not enable -ffast-math globally: it permits assumptions that NaN/Inf never occur.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -fno-fast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fno-fast-math")

aux_source_directory(${PROJECT_SOURCE_DIR}/ SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/app/ SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/app/gyro SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/app/display SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/common/src SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/drv/src SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../example/src SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../user_app/src SRC)

# 运行时安全层以 #include "my_motor.cpp" 的方式复用既有控制律。
# 因此必须从独立编译单元中移除原始 my_motor.cpp，避免同一符号定义两次。
set(LQ_RUNTIME_SAFETY_WRAPPER "${PROJECT_SOURCE_DIR}/../user_app/src/my_motor_runtime_patch.cpp")
if (EXISTS "${LQ_RUNTIME_SAFETY_WRAPPER}")
    list(REMOVE_ITEM SRC "${PROJECT_SOURCE_DIR}/../user_app/src/my_motor.cpp")
    message(STATUS "Runtime safety wrapper enabled: ${LQ_RUNTIME_SAFETY_WRAPPER}")
endif()

# 指定所用的头文件路径
include_directories(
    ${PROJECT_SOURCE_DIR}/
    ${PROJECT_SOURCE_DIR}/../libraries/app/
    ${PROJECT_SOURCE_DIR}/../libraries/app/gyro
    ${PROJECT_SOURCE_DIR}/../libraries/app/display
    ${PROJECT_SOURCE_DIR}/../libraries/common/inc
    ${PROJECT_SOURCE_DIR}/../libraries/drv/inc
    ${PROJECT_SOURCE_DIR}/../example/inc
    ${PROJECT_SOURCE_DIR}/../user_app/include
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)
pkg_check_modules(NCNN REQUIRED ncnn)
if(NCNN_FOUND)
    add_definitions(-DLQ_HAVE_OPENCV)
endif()

link_directories(${PROJECT_SOURCE_DIR}/../libraries/drv/src/)

add_executable(${PROJECT_NAME} ${SRC})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/../libraries/drv/inc
    ${OPENCV_INCLUDE_DIRS}
    ${NCNN_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    pthread
    rt
    lqncnn
    lq_camera_ex
    ${NCNN_LDFLAGS}
    ${OPENCV_LDFLAGS}
)
