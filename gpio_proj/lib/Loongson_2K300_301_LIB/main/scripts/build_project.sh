#!/bin/bash

# 生成的 CMake 宏文件（main目录下）
TOOLCHAIN_CMAKE_MACRO_FILE="./toolchain_path.cmake"

##################################################
# 函数作用：生成 CMake 可识别的宏文件
# 参数说明：无
# 使用说明：内部使用，不用管
##################################################
function gen_cmake_macro_file() {
    log_info "================================================================== CMake 宏文件生成 =================================================================="
    log_info "🔧 开始生成 CMake 宏文件：${TOOLCHAIN_CMAKE_MACRO_FILE}"
    # 写入绝对路径宏（CMAKE_TOOLCHAIN_PATH 供 CMake 调用）
    cat > "${TOOLCHAIN_CMAKE_MACRO_FILE}" << EOF
    set(CMAKE_TOOLCHAIN_PATH "${toolchain_path}" CACHE PATH "Loongson toolchain path" FORCE)
EOF
    # 验证宏文件是否生成
    if [ -f "${TOOLCHAIN_CMAKE_MACRO_FILE}" ]; then
        log_info "✅ CMake 宏文件生成成功！内容："
        cat "${TOOLCHAIN_CMAKE_MACRO_FILE}" | grep -v "^#" | grep -v "^$"
    else
        log_error "❌ CMake 宏文件生成失败！"
    fi
    log_info "======================================================================================================================================================\n"
}

##################################################
# 函数作用：构建编译项目
# 参数说明：
#       $1 : 构建目录名称
# 使用说明：内部使用，不用管
##################################################
function compile_project() { 
    log_info "====================================================================== 编译项目 ======================================================================"
    # 源文件列表文件路径
    local source_list_file="$1/.source_list.txt"
    # 检查构建目录是否存在
    if [ ! -d "$1" ]; then
        # 第一次编译：构建 + 编译
        log_info "🔧 首次编译：创建并配置构建目录：$1"
        cmake -B "$1" || log_error "❌ cmake 配置失败！"
        # 记录当前源文件列表
        find ./.. -type f -name "*.cpp" -o -name "*.c" -o -name "*.cc" | sort > "$source_list_file"
        log_info "✅ 记录源文件列表到：$source_list_file"
    else
        # 非首次编译
        if [ ! -f "$source_list_file" ]; then
            # 源文件列表不存在，重新构建
            log_info "🔧 源文件列表不存在，重新构建：$1"
            rm -rf "$1"
            cmake -B "$1" || log_error "❌ cmake 配置失败！"
            find ./.. -type f -name "*.cpp" -o -name "*.c" -o -name "*.cc" | sort > "$source_list_file"
            log_info "✅ 记录源文件列表到：$source_list_file"
        else
            # 比较源文件列表
            local current_sources=$(find ./.. -type f -name "*.cpp" -o -name "*.c" -o -name "*.cc" | sort)
            local old_sources=$(cat "$source_list_file")
            if [ "$current_sources" != "$old_sources" ]; then
                # 源文件有变化，重新构建
                log_info "🔧 源文件有变化，重新构建：$1"
                rm -rf "$1"
                cmake -B "$1" || log_error "❌ cmake 配置失败！"
                find ./.. -type f -name "*.cpp" -o -name "*.c" -o -name "*.cc" | sort > "$source_list_file"
                log_info "✅ 更新源文件列表到：$source_list_file"
            else
                # 源文件无变化，直接编译
                log_info "🔧 源文件无变化，执行增量编译：$1"
            fi
        fi
    fi
    # 编译项目
    log_info "🔧 开始编译项目（线程数：$(nproc)）"
    cmake --build "$1" -j"$(nproc)" || log_error "❌ 编译失败！"
    log_info "✅ 项目编译完成！"
    log_info "======================================================================================================================================================\n"
}
