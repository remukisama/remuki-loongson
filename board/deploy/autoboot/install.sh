#!/bin/sh
# ============================================================
#  龙芯 2K0301 智能小车 — 开机自启安装脚本
#
#  用法：
#    chmod +x install.sh
#    ./install.sh          # 安装
#    ./install.sh remove   # 卸载
#
#  安装方式：文件夹不动，service 文件索引当前目录
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVICE_DIR="/etc/systemd/system"

# ==================== 卸载 ====================
if [ "$1" = "remove" ]; then
    echo "[Uninstall] Removing GPIO auto-start service..."

    systemctl stop gpio-manager.service 2>/dev/null
    systemctl disable gpio-manager.service 2>/dev/null
    rm -f ${SERVICE_DIR}/gpio-manager.service

    systemctl daemon-reload 2>/dev/null
    echo "[Uninstall] Done"
    exit 0
fi

# ==================== 检查文件 ====================
echo "========================================"
echo "  Loongson 2K301 Auto-Start Installer"
echo "========================================"
echo ""
echo "Install dir: ${SCRIPT_DIR}"
echo ""

for f in "${SCRIPT_DIR}/bin/gpio_manager"; do
    if [ ! -f "$f" ]; then
        echo "[ERROR] Missing: $f"
        exit 1
    fi
done
echo "[OK] All files present"

# ==================== 设置权限 ====================
echo "[1/3] Setting permissions..."
chmod +x ${SCRIPT_DIR}/bin/gpio_manager

# ==================== 生成 service 文件（替换路径）====================
echo "[2/3] Installing gpio-manager systemd service..."
sed "s|INSTALL_DIR|${SCRIPT_DIR}|g" ${SCRIPT_DIR}/service/gpio-manager.service > ${SERVICE_DIR}/gpio-manager.service
systemctl daemon-reload 2>/dev/null

# ==================== 启用 + 启动 ====================
echo "[3/3] Enabling and starting..."
systemctl enable gpio-manager.service 2>/dev/null
systemctl disable screen.service 2>/dev/null || true
systemctl stop screen.service 2>/dev/null || true

systemctl start gpio-manager.service
sleep 1

# ==================== 状态 ====================
echo ""
echo "========================================"
echo "  Installation Complete"
echo "========================================"
echo ""
systemctl status gpio-manager.service --no-pager 2>/dev/null | head -5
echo ""
echo "Commands:"
echo "  查看看门狗日志:  journalctl -u gpio-manager -f"
echo "  停止看门狗:      systemctl stop gpio-manager"
echo "  卸载看门狗:      ./install.sh remove"
