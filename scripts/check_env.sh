#!/bin/bash
###############################################################################
# 环境自检脚本（在 Linux 编译机上运行）
#
#   bash scripts/check_env.sh              # 只检查主机环境与依赖
#   bash scripts/check_env.sh 192.168.1.100 # 额外检查板卡连通性与目标文件
#
# 退出码：0 全部就绪 / 1 存在阻断项
###############################################################################

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLCHAIN_DIR="${ROOT}/tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
TOOLCHAIN_TAR="${ROOT}/tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6.tar.xz"
DEP_DIR="${ROOT}/tools/LQ_Dep_libs"
BOARD_IP="${1:-}"

G='\033[32m'; Y='\033[33m'; R='\033[31m'; N='\033[0m'
FAIL=0
ok()   { echo -e "  ${G}[ OK ]${N} $1"; }
warn() { echo -e "  ${Y}[WARN]${N} $1"; }
bad()  { echo -e "  ${R}[FAIL]${N} $1"; FAIL=$((FAIL+1)); }

echo "== 1. 主机与基础工具 =="
if [ "$(uname -s)" = "Linux" ]; then ok "Linux ($(uname -m))"; else bad "非 Linux 主机，交叉编译无法在本机进行"; fi

for c in cmake pkg-config make nproc ssh scp tar; do
    if command -v "$c" >/dev/null 2>&1; then
        ok "$c -> $(command -v "$c")"
    else
        bad "$c 未安装（Ubuntu: sudo apt install -y $c）"
    fi
done

echo
echo "== 2. 交叉工具链 =="
if [ -x "${TOOLCHAIN_DIR}/bin/loongarch64-linux-gnu-g++" ]; then
    ok "工具链就绪：$(  "${TOOLCHAIN_DIR}/bin/loongarch64-linux-gnu-g++" --version | head -1 )"
elif [ -f "${TOOLCHAIN_TAR}" ]; then
    warn "只有压缩包，首次执行 build.sh 会自动解压：$(basename "${TOOLCHAIN_TAR}")"
else
    bad "缺少工具链目录或压缩包：tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
fi

echo
echo "== 3. 目标平台依赖库 =="
if [ -d "${DEP_DIR}" ]; then
    for lib in opencv_install ncnn_install ffmpeg_install; do
        if [ -d "${DEP_DIR}/${lib}/lib/pkgconfig" ]; then
            cnt=$(find "${DEP_DIR}/${lib}/lib/pkgconfig" -name '*.pc' | wc -l)
            ok "${lib}（${cnt} 个 .pc）"
        elif [ -f "${DEP_DIR}/${lib}.tar.xz" ]; then
            warn "${lib} 仅压缩包，build.sh 会自动解压"
        else
            bad "${lib} 缺失（目录或 ${lib}.tar.xz）"
        fi
    done
    if [ -d "${DEP_DIR}/opencv_install/lib/pkgconfig" ]; then
        export PKG_CONFIG_PATH="${DEP_DIR}/opencv_install/lib/pkgconfig:${DEP_DIR}/ncnn_install/lib/pkgconfig:${DEP_DIR}/ffmpeg_install/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
        for p in opencv4 ncnn libavformat; do
            pkg-config --exists "$p" 2>/dev/null && ok "pkg-config 可解析 $p" || warn "pkg-config 暂时解析不到 $p（工具链 sysroot 下运行时以 build.sh 输出为准）"
        done
    fi
else
    bad "缺少 tools/LQ_Dep_libs/（见 tools/README.md）"
fi

echo
echo "== 4. 工程完整性 =="
for f in main/CMakeLists.txt main/build.sh gpio_proj/CMakeLists.txt \
         libraries/common/inc libraries/drv/inc \
         vendor/loongson_sdk_V2.1.0/libraries/common/src/lq_base_class.cpp; do
    [ -e "${ROOT}/${f}" ] && ok "$f" || bad "$f 缺失"
done

echo
echo "== 5. gpio_proj 依赖解析 =="
if [ -f "${ROOT}/gpio_proj/lib/Loongson_2K300_301_LIB/libraries/common/src/lq_base_class.cpp" ]; then
    ok "用仓库内旧副本 gpio_proj/lib/Loongson_2K300_301_LIB/libraries"
elif [ -f "${ROOT}/vendor/loongson_sdk_V2.1.0/libraries/common/src/lq_base_class.cpp" ]; then
    ok "用 vendor/loongson_sdk_V2.1.0/libraries"
else
    bad "父仓 libraries/ 缺 lq_base_class.cpp，gpio_proj 无法编译（需补 vendor SDK）"
fi

if [ -n "${BOARD_IP}" ]; then
    echo
    echo "== 6. 板卡连通性（${BOARD_IP}）=="
    if ping -c 2 -W 3 "${BOARD_IP}" >/dev/null 2>&1; then
        ok "ping 通"
        if ssh -o ConnectTimeout=5 -o BatchMode=yes "root@${BOARD_IP}" "echo ok" >/dev/null 2>&1; then
            ok "SSH 免密可登录"
            ssh -o ConnectTimeout=5 "root@${BOARD_IP}" \
                "test -x /root/gpio_manager && echo HAS_MANAGER || echo NO_MANAGER; \
                 pgrep -x gpio_manager >/dev/null && echo MANAGER_RUNNING || echo MANAGER_STOPPED" \
                2>/dev/null | while read -r line; do
                case "$line" in
                    HAS_MANAGER) ok "板端已有 gpio_manager" ;;
                    NO_MANAGER) warn "板端尚无 gpio_manager（先部署看门狗）" ;;
                    MANAGER_RUNNING) ok "gpio_manager 正在运行" ;;
                    MANAGER_STOPPED) warn "gpio_manager 未运行（--mode=car 会被拒绝启动）" ;;
                esac
            done
        else
            warn "SSH 不可免密登录（需配置公钥，或手动输入密码测试）"
        fi
    else
        bad "ping 不通：检查网段是否为 192.168.1.x / 板卡是否上电"
    fi
fi

echo
if [ "${FAIL}" -eq 0 ]; then
    echo -e "${G}环境就绪，可以执行：cd main && ./build.sh${N}"
    exit 0
else
    echo -e "${R}存在 ${FAIL} 项阻断问题，按上面 [FAIL] 逐条处理后重跑。${N}"
    exit 1
fi
