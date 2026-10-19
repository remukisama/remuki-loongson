#!/bin/bash
# 驱动模块批量发送总控脚本
# 功能：遍历所有子目录，调用子目录的 scp_send.sh 脚本发送.ko文件到指定IP
# 使用方法：
#   1. 赋予执行权限：chmod +x send_all_ko.sh
#   2. 基础用法：./send_all_ko.sh 目标IP [远程接收目录]
#   3. 示例：./send_all_ko.sh 192.168.1.100 /root/drivers/

# ===================== 配置项（可按需修改）=====================
# 子目录中发送脚本的名称（固定为scp_send.sh）
SEND_SCRIPT_NAME="scp_send.sh"
# 跳过的子目录（无需发送的目录，用空格分隔）
SKIP_DIRS="docs scripts temp"
# =================================================================

# 颜色输出函数
red_echo() { echo -e "\033[31m$1\033[0m"; }
green_echo() { echo -e "\033[32m$1\033[0m"; }
yellow_echo() { echo -e "\033[33m$1\033[0m"; }
blue_echo() { echo -e "\033[34m$1\033[0m"; }
bold_echo() { echo -e "\033[1m$1\033[0m"; }

# 显示用法
show_usage() {
    echo "用法：$0 <目标IP地址> [远程接收目录]"
    echo "说明："
    echo "  1. 目标IP地址为必传参数，远程接收目录可选（默认使用子脚本的默认目录）"
    echo "  2. 脚本会自动遍历所有子目录，调用其中的 ${SEND_SCRIPT_NAME} 发送.ko文件"
    echo "示例："
    echo "  $0 192.168.1.100"
    echo "  $0 192.168.1.100 /home/root/drivers"
    exit 1
}

# 检查参数：至少传入目标IP
if [ $# -lt 1 ]; then
    red_echo "❌ 错误：缺少目标IP地址参数！"
    show_usage
fi

# 解析参数
TARGET_IP=$1
REMOTE_DIR=$2  # 可选参数，未传则为空

# 获取所有子目录（排除当前目录、隐藏目录、跳过目录）
SUBDIRS=$(find . -maxdepth 1 -type d ! -name "." ! -name ".." ! -name ".*" | sort)

# 遍历子目录执行发送
send_success_count=0
send_fail_count=0
total_dir_count=0

green_echo "===================================================================================================="
blue_echo "开始遍历子目录并执行发送..."
green_echo "====================================================================================================\n"

for dir in $SUBDIRS; do
    # 提取目录名（如 ./lq_i2c_icm42688_drv → lq_i2c_icm42688_drv）
    dir_name=$(basename "$dir")
    
    # 跳过指定目录
    if [[ " $SKIP_DIRS " =~ " $dir_name " ]]; then
        yellow_echo "⚠️  跳过目录：$dir_name（已配置为跳过）"
        continue
    fi

    total_dir_count=$((total_dir_count + 1))
    script_path="$dir/$SEND_SCRIPT_NAME"

    # 检查子目录是否有发送脚本
    if [ ! -f "$script_path" ]; then
        yellow_echo "⚠️  目录 $dir_name 无 ${SEND_SCRIPT_NAME} 脚本，跳过发送"
        send_fail_count=$((send_fail_count + 1))
        continue
    fi

    # 检查脚本是否有执行权限
    if [ ! -x "$script_path" ]; then
        red_echo "❌ 目录 $dir_name 的 ${SEND_SCRIPT_NAME} 无执行权限，尝试添加..."
        chmod +x "$script_path"
        if [ $? -ne 0 ]; then
            red_echo "❌ 目录 $dir_name 添加执行权限失败，跳过发送"
            send_fail_count=$((send_fail_count + 1))
            continue
        fi
    fi

    # 执行子目录发送脚本
    blue_echo "📂 正在处理目录：$dir_name"
    cd "$dir" || {
        red_echo "❌ 进入目录 $dir_name 失败，跳过发送"
        send_fail_count=$((send_fail_count + 1))
        cd - > /dev/null
        continue
    }

    # 调用子脚本（传递IP和可选的远程目录）
    if [ -n "$REMOTE_DIR" ]; then
        ./${SEND_SCRIPT_NAME} "$TARGET_IP" "$REMOTE_DIR"
    else
        ./${SEND_SCRIPT_NAME} "$TARGET_IP"
    fi

    # 检查执行结果
    if [ $? -eq 0 ]; then
        green_echo "✅ 目录 $dir_name 发送成功！\n\n"
        send_success_count=$((send_success_count + 1))
    else
        red_echo "❌ 目录 $dir_name 发送失败！\n\n"
        send_fail_count=$((send_fail_count + 1))
    fi

    cd - > /dev/null
done

# 发送结果统计
bold_echo "\n===================================================================================================="
bold_echo "📊 批量发送结果统计"
bold_echo "===================================================================================================="
echo "  总处理的目录数：$total_dir_count"
green_echo "  发送成功的数量：$send_success_count 个目录"
red_echo "  发送失败或跳过：$send_fail_count 个目录"
echo ""
if [ $send_fail_count -eq 0 ]; then
    green_echo "🎉 所有驱动模块批量发送完成！"
else
    yellow_echo "⚠️  批量发送完成，但有部分目录发送失败（详见上方日志）"
fi

exit 0