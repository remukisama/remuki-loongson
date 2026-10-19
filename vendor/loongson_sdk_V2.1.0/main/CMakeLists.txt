cmake_minimum_required(VERSION 3.16)    # 限制最低CMake版本

# 设置交叉编译工具路径
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/toolchain_path.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/toolchain_path.cmake")
endif()

# 指定交叉编译工具所使用的C语言编译器
set(CMAKE_C_COMPILER "${CMAKE_TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-gcc")
# 指定交叉编译工具所使用的C++编译器
set(CMAKE_CXX_COMPILER "${CMAKE_TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-g++")

# 项目名称
set(PROJECT_NAME "main")

project(${PROJECT_NAME})

# 指定所用的源文件路径(与 file(GLOB LIB ${XXX}) 功能相同，但可将所有参数保存到一个变量中)
aux_source_directory(${PROJECT_SOURCE_DIR}/ SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/app/ SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/app/gyro SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/app/display SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/common/src SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../libraries/drv/src SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../example/src SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../user_app SRC)

# 指定所用的头文件路径
include_directories(
    ${PROJECT_SOURCE_DIR}/
    ${PROJECT_SOURCE_DIR}/../libraries/app/
    ${PROJECT_SOURCE_DIR}/../libraries/app/gyro
    ${PROJECT_SOURCE_DIR}/../libraries/app/display
    ${PROJECT_SOURCE_DIR}/../libraries/common/inc
    ${PROJECT_SOURCE_DIR}/../libraries/drv/inc
    ${PROJECT_SOURCE_DIR}/../example/inc
    ${PROJECT_SOURCE_DIR}/../user_app
)

# 查找 pkgconfig 工具（必须，用于解析 .pc 文件）
find_package(PkgConfig REQUIRED)

# 查找 OpenCV 动态库
pkg_check_modules(OPENCV REQUIRED opencv4)

# 查找 ncnn 动态库
pkg_check_modules(NCNN REQUIRED ncnn)
if(NCNN_FOUND)
    add_definitions(-DLQ_HAVE_OPENCV)
endif()

# 查找 ffmpeg 动态库
# pkg_check_modules(FFMPEG REQUIRED libavformat libavcodec libavdevice libavutil)

link_directories(
    ${PROJECT_SOURCE_DIR}/../libraries/drv/src/  # 静态库的存放路径
)

# 由源文件生成一个可执行程序
add_executable(${PROJECT_NAME} ${SRC})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/../libraries/drv/inc  # ncnn 头文件路径
    ${OPENCV_INCLUDE_DIRS}                      # OpenCV 头文件路径
    ${NCNN_INCLUDE_DIRS}                        # ncnn 头文件路径
    # ${FFMPEG_INCLUDE_DIRS}  # ffmpeg 头文件路径
)

# 连接库
target_link_libraries(${PROJECT_NAME} PUBLIC
    pthread
    lqncnn
    lq_camera_ex
    ${NCNN_LDFLAGS}
    ${OPENCV_LDFLAGS}
    # ${FFMPEG_LDFLAGS}
)
