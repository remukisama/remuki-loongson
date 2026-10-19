#!/bin/bash

# 交叉编译工具链配置
TOOLCHAIN_DIR_NAME="loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"          # 工具链目录名
TOOLCHAIN_TAR_NAME="loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6.tar.xz"   # 工具链压缩包名

##################################################
# 函数作用：处理交叉编译工具链
# 参数说明：
#       $1 : 工具所在目录
# 使用说明：内部使用，不用管
##################################################
function deal_cross_toolchain() {
    # 配置交叉编译工具链
    log_info "================================================================= 处理交叉编译工具链 ================================================================="
    # 拼接工具链路径
    toolchain_dir_full="$1/${TOOLCHAIN_DIR_NAME}"
    toolchain_tar_full="$1/${TOOLCHAIN_TAR_NAME}"

    log_info "🔍 开始检查 tools 目录下的交叉编译工具链：${toolchain_dir_full}"

    # 检查工具链目录是否存在
    if [ -d "${toolchain_dir_full}" ]; then
        log_info "✅ tools 目录下找到交叉编译工具链：${toolchain_dir_full}"
        toolchain_path="${toolchain_dir_full}"
    else
        log_warn "⚠️ tools 目录下未找到交叉编译工具链，检查压缩包：${toolchain_tar_full}"

        # 检查压缩包是否存在
        if [ -f "${toolchain_tar_full}" ]; then
            log_info "✅ 找到交叉编译工具链压缩包，开始解压到 tools 目录下..."
            # 解压压缩包到 tools 目录下（保留权限，显示进度）
            tar -xvf "${toolchain_tar_full}" -C "$1" || log_error "❌ 解压交叉编译工具链压缩包失败！"
            log_info "✅ 解压交叉编译工具链完成：${TOOLCHAIN_TAR_NAME} --> ${toolchain_dir_full}"
            # 验证解压后的交叉编译工具链目录
            if [ -d "${toolchain_dir_full}" ]; then
                toolchain_path="${toolchain_dir_full}"
                log_info "✅ 解压后找到交叉编译工具链目录：${toolchain_path}"
            else
                log_error "❌ 解压后未找到交叉编译工具链目录 ${TOOLCHAIN_DIR_NAME}，请检查压缩包内容！"
            fi
        else
            # 文件夹和压缩包都不存在
            log_error "❌ 交叉编译工具链和压缩包都不存在！
            预期目录：${toolchain_dir_full}
            预期压缩包：${toolchain_tar_full}
            请将压缩包放到 tools 目录下，或手动解压到 tools 目录下！"
        fi
    fi
    log_info "======================================================================================================================================================\n"
}
