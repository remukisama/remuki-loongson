#!/bin/bash

# 所以库统一的pkgconfig路径
PKG_REL_PATH="lib/pkgconfig/"

##################################################
# 函数作用：处理常驻依赖库目录
# 参数说明：
#       $1 : 工具所在目录
#       $2 : 依赖库所在目录
# 使用说明：内部使用，不用管
##################################################
function handle_resident_lib_dir() {
    log_info "================================================================= 处理常驻依赖库目录 ================================================================="
    target_dir_full="$1/$2"
    log_info "🔍 检测常驻依赖库目录：${target_dir_full}"
    # 检查LQ_Dep_libs常驻目录是否存在
    if [ ! -d "${target_dir_full}" ]; then
        log_error "❌ 常驻依赖库目录 $2 不存在！
        预期路径：${target_dir_full}
        请先创建该目录：mkdir -p ${target_dir_full}"
    fi
    log_info "✅ 找到常驻依赖库目录：${target_dir_full}"
    log_info "======================================================================================================================================================\n"
}

##################################################
# 函数作用：处理单个依赖库
# 参数说明：
#       $1 : 依赖库目录名
#       $2 : 目标依赖库根目录
# 使用说明：内部使用，不用管，想要添加依赖库直接在
#         DEP_LIBS 和 PKG_CONFIG_NAMES 变量中添加即可
##################################################
function process_single_dep_lib() {
    local lib_dir_name=$1       # 依赖库目录名
    local target_dir_full=$2    # 目标依赖库根目录
    local lib_dir_path="${target_dir_full}/${lib_dir_name}"         # 依赖库目录路径
    local lib_tar_path="${target_dir_full}/${lib_dir_name}.tar.xz"   # 依赖库压缩包路径
    # 检测文件夹是否存在
    if [ -d "${lib_dir_path}" ]; then
        log_info "✅ 找到 ${lib_dir_name} 文件夹：${lib_dir_path}"
        return 0
    fi
    # 检测压缩包并解压
    log_warn "⚠️ 未找到 ${lib_dir_name} 文件夹，检查压缩包：${lib_tar_path}"
    if [ ! -f "${lib_tar_path}" ]; then
        log_error "❌ ${lib_dir_name} 的文件夹和压缩包都不存在！
        预期文件夹：${lib_dir_path}
        预期压缩包：${lib_tar_path}"
    fi
    log_info "🔧 开始解压 ${lib_dir_name} 压缩包..."
    if ! tar -xvf "${lib_tar_path}" -C "${target_dir_full}"; then
        log_error "❌ 解压 ${lib_dir_name} 压缩包失败！"
    fi
    log_info "✅ 解压完成：${lib_tar_path} --> ${lib_dir_path}"
    # 验证解压结果
    if [ ! -d "${lib_dir_path}" ]; then
        log_error "❌ 解压后仍未找到 ${lib_dir_name} 文件夹，请检查压缩包内容！"
    fi
}

##################################################
# 函数作用：遍历所有依赖库
# 参数说明：
#       $1 : 任意数量的依赖目录
# 使用说明：内部使用，不用管
##################################################
function traverse_process_dep_libs() {
    local dep_libs=$1

    log_info "=================================================================== 遍历处理依赖库 ==================================================================="
    current_idx=0
    total_libs=$(echo ${dep_libs} | tr ' ' '\n' | wc -l)
    for lib_dir_name in $(echo ${dep_libs} | tr ' ' '\n'); do
        current_idx=$((current_idx + 1))
        log_info "🔧 开始处理依赖库：${lib_dir_name}"
        # 仅获取函数返回的纯路径（日志已输出到标准错误，不会污染）（函数）
        process_single_dep_lib "${lib_dir_name}" "${target_dir_full}"
        # 保存每个库的纯路径
        eval "${lib_dir_name}_PATH='${target_dir_full}/${lib_dir_name}/'"
        eval "lib_full_path=\${${lib_dir_name}_PATH}"
        if [ ${current_idx} -eq ${total_libs} ]; then
            log_info "✅ ${lib_dir_name} 路径保存成功：\$${lib_dir_name}_PATH = ${lib_full_path}"
        else
            log_info "✅ ${lib_dir_name} 路径保存成功：\$${lib_dir_name}_PATH = ${lib_full_path}\n"
        fi
    done
    log_info "======================================================================================================================================================\n"
}

##################################################
# 函数作用：通用 PKG_CONFIG_PATH 配置函数
# 参数说明：
#       $1 : 任意数量的依赖目录
#       $2 : 任意数量的各类库检测名
# 使用说明：内部使用，不用管
##################################################
function setup_pkgconfig_path() {
    log_info "================================================================ 配置 PKG_CONFIG_PATH ================================================================"
    local pkg_path_list=""
    # 拆分依赖库列表和检测名列表
    local i=0
    local lib_dir_name=""
    local pkg_check_name=""
    # 遍历依赖库列表
    for lib_dir_name in $(echo $1 | tr ' ' '\n'); do
        # 按索引取对应的检测名
        pkg_check_name=$(echo $2 | cut -d' ' -f$((i+1)))
        
        # 获取纯路径变量（无日志污染）
        eval "lib_full_path=\${${lib_dir_name}_PATH}"
        log_info "🔍 处理 ${lib_dir_name}：路径 = ${lib_full_path}"
        
        # 拼接pkgconfig路径
        local pkg_dir="${lib_full_path}/${PKG_REL_PATH}"
        pkg_dir=$(realpath -m "${pkg_dir}" 2>/dev/null || echo "${pkg_dir}")
        log_info "🔍 ${lib_dir_name} pkgconfig 路径 = ${pkg_dir}"
        
        # 验证pkgconfig目录
        if [ ! -d "${pkg_dir}" ]; then
            log_error "❌ ${lib_dir_name} 的 pkgconfig 目录不存在！
            实际路径：${pkg_dir}
            请检查 ${lib_full_path} 下是否有 ${PKG_REL_PATH} 目录！"
        fi
        log_info "✅ 找到 ${lib_dir_name} 的 pkgconfig 目录：${pkg_dir}\n"
        pkg_path_list+="${pkg_dir}:"
        i=$((i+1))
    done
    # 配置并去重 PKG_CONFIG_PATH
    export PKG_CONFIG_PATH=$(echo -e "${pkg_path_list}${PKG_CONFIG_PATH:-}" | tr  ':' '\n' | sort -u | tr '\n' ':' | sed 's/:$//')
    log_info "✅ 配置完成 PKG_CONFIG_PATH：${PKG_CONFIG_PATH}"
    log_info "======================================================================================================================================================\n"
    # 验证每个库的pkg-config
    log_info "=============================================================== 验证 pkg-config 可用性 ==============================================================="
    for pkg_check_name in $2; do
        if pkg-config --exists "${pkg_check_name}" 2>/dev/null; then
            log_info "✅ pkg-config 验证成功：${pkg_check_name}"
            log_info "  ├─ 编译参数：$(pkg-config --cflags ${pkg_check_name})"
            log_info "  └─ 链接参数：$(pkg-config --libs ${pkg_check_name})\n"
        else
            log_warn "⚠️ pkg-config 未找到 ${pkg_check_name} 库（非致命错误，继续执行）"
            # 改为警告而非终止，避免单个库问题导致脚本中断
        fi
    done
    log_info "🎉 PKG_CONFIG_PATH 配置 & 验证完毕!"
    log_info "======================================================================================================================================================\n"
}
