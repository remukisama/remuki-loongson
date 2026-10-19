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

set(LQ_USER_APP_SOURCES
    ${PROJECT_SOURCE_DIR}/../user_app/src/camera_config_safety.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/encoder.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/gpio_protect_heartbeat.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/image.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/image_cv.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/motion_feedback_guard_threadsafe.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/my_cascade_pid.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/my_motor_runtime_patch.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/my_mpu6050.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/ncnn_inference.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/red_marker_detect.cpp
    ${PROJECT_SOURCE_DIR}/../user_app/src/roundabout_controller.cpp
)

set(LQ_RUNTIME_HELPER_SOURCES
    ${PROJECT_SOURCE_DIR}/../example/src/24_lq_line_patrol_debug_demo.cpp
    ${PROJECT_SOURCE_DIR}/../example/src/37_lq_motor_pwm_encoder_diag.cpp
    ${PROJECT_SOURCE_DIR}/../example/src/36_lq_vision_capture_demo.cpp
    ${PROJECT_SOURCE_DIR}/../example/src/38_lq_udp_llm_pid_autotune_demo.cpp
    ${PROJECT_SOURCE_DIR}/../example/src/39_lq_motor_id_demo.cpp
    ${PROJECT_SOURCE_DIR}/../example/src/40_lq_pwm_quick_test.cpp
    ${PROJECT_SOURCE_DIR}/../example/src/41_lq_gyro_rate_pid_tune_demo.cpp
    ${PROJECT_SOURCE_DIR}/../example/src/42_lq_gyro_motor_pid_tune_demo.cpp
    ${PROJECT_SOURCE_DIR}/../example/src/43_lq_diff_speed_test_demo.cpp
    ${PROJECT_SOURCE_DIR}/../example/src/44_lq_channel_check_demo.cpp
    ${PROJECT_SOURCE_DIR}/../example/src/45_lq_icm42605_test_demo.cpp
)

list(APPEND SRC ${LQ_USER_APP_SOURCES} ${LQ_RUNTIME_HELPER_SOURCES})

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
