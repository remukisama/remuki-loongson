#!/bin/bash
# 驱动模块发送脚本（子文件夹专用）
# 功能：检测当前目录ko文件夹下的.ko文件，发送到指定IP的目标目录
# 使用方法：./send_ko.sh 目标IP [目标目录]
# 示例：
#   ./send_ko.sh 192.168.1.100          # 发送到默认目录 /root/drivers/
#   ./send_ko.sh 192.168.1.100 /tmp/   # 发送到指定目录 /tmp/

# ===================== 配置项（可根据需求修改）=====================
# 本地.ko文件存放目录（相对于脚本所在目录）
LOCAL_KO_DIR="./ko"
# 远程默认接收目录（未指定时使用）
REMOTE_DEFAULT_DIR="/home/root/LQ_Drv_libs"
# 远程登录用户名（根据目标设备修改）
REMOTE_USER="root"
# =================================================================

# 颜色输出函数
function red_echo()    { echo -e "\033[31m$1\033[0m"; }
function green_echo()  { echo -e "\033[32m$1\033[0m"; }
function yellow_echo() { echo -e "\033[33m$1\033[0m"; }
function blue_echo()   { echo -e "\033[34m$1\033[0m"; }

# 显示用法
function show_usage() {
    yellow_echo "===================================================================================================="
    yellow_echo "用法：$0 <目标IP地址> [远程接收目录]"
    yellow_echo "示例："
    yellow_echo "  发送到默认目录：$0 192.168.1.100"
    yellow_echo "  发送到指定目录：$0 192.168.1.100 /home/root/drivers"
    yellow_echo "===================================================================================================="
    exit 1
}

# 检查参数：至少传入IP地址
if [ $# -lt 1 ]; then
    red_echo "❌ 错误：缺少目标IP地址参数！"
    show_usage
fi

# 解析参数
TARGET_IP=$1
REMOTE_DIR=${2:-$REMOTE_DEFAULT_DIR}  # 未指定则用默认目录

# 检查本地ko目录是否存在
if [ ! -d "$LOCAL_KO_DIR" ]; then
    yellow_echo "⚠️ 警告：本地ko目录 $LOCAL_KO_DIR 不存在，尝试检测当前目录的.ko文件..."
    LOCAL_KO_DIR="."  # 切换到当前目录检测
fi

# 检测.ko文件
KO_FILES=$(find "$LOCAL_KO_DIR" -maxdepth 1 -name "*.ko" -type f)
if [ -z "$KO_FILES" ]; then
    red_echo "❌ 错误：在 $LOCAL_KO_DIR 目录下未找到任何.ko文件！"
    exit 1
fi

# 显示检测到的文件
green_echo "===================================================================================================="
green_echo "🔍 检测到以下.ko文件："
for ko in $KO_FILES; do
    echo "  - $(basename $ko)"
done
green_echo "===================================================================================================="

# 创建远程目录（防止目录不存在）
blue_echo "✅ 第一步：创建远程目录 $REMOTE_DIR..."
ssh "$REMOTE_USER@$TARGET_IP" "mkdir -p $REMOTE_DIR"
if [ $? -ne 0 ]; then
    red_echo "❌ 错误：创建远程目录失败！请检查IP/用户名/网络连接。"
    exit 1
fi

# 发送.ko文件（使用scp）
blue_echo "✅ 第二步：开始发送.ko文件到 $TARGET_IP..."
for ko in $KO_FILES; do
    scp -O "$ko" "$REMOTE_USER@$TARGET_IP:$REMOTE_DIR"
    if [ $? -eq 0 ]; then
        green_echo "✅ 成功发送：$(basename $ko)"
    else
        red_echo "❌ 发送失败：$(basename $ko)"
        exit 1
    fi
done

# 发送完成提示
green_echo "========================================"
green_echo "🎉 所有.ko文件已成功发送到："
green_echo "  远程地址：$REMOTE_USER@$TARGET_IP"
green_echo "  目标目录：$REMOTE_DIR"
green_echo "========================================\n"

exit 0
