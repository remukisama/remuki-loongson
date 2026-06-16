#!/bin/bash

##############################################################################
# gpio_proj 独立编译部署脚本
#
# 使用方法：
#   ./build.sh                    # 编译所有
#   ./build.sh manager            # 只编译守护进程
#   ./build.sh test               # 只编译测试程序
#   ./build.sh manager <IP>       # 编译 + 传输到板子
#   ./build.sh manager <IP> run   # 编译 + 传输 + 运行
##############################################################################

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_TOOLCHAIN_PATH="${SCRIPT_DIR}/../tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
if [ ! -x "${DEFAULT_TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-g++" ]; then
    DEFAULT_TOOLCHAIN_PATH="/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
fi
TOOLCHAIN_PATH="${TOOLCHAIN_PATH:-${DEFAULT_TOOLCHAIN_PATH}}"
BOARD_IP="${BOARD_IP:-}"
BOARD_USER="${BOARD_USER:-root}"
BOARD_TARGET_PATH="${BOARD_TARGET_PATH:-~}"
BUILD_DIR="build"
BOARD_TYPE="2K301"

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'
log_info()  { echo -e "${GREEN}[$(date +%H:%M:%S)] [INFO] $1${NC}"; }
log_error() { echo -e "${RED}[$(date +%H:%M:%S)] [ERROR] $1${NC}"; exit 1; }

# 参数解析
BUILD_TARGET="all"
BOARD_IP_ARG=""
RUN_FLAG=""

for arg in "$@"; do
    case $arg in
        manager|test|all)
            BUILD_TARGET="$arg" ;;
        run|--run|-r)
            RUN_FLAG="1" ;;
        *)
            if [[ $arg =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                BOARD_IP_ARG="$arg"
            fi ;;
    esac
done

[ -n "$BOARD_IP_ARG" ] && BOARD_IP="$BOARD_IP_ARG"

# 编译
do_build() {
    log_info "Building gpio_proj (target=$1, board=$BOARD_TYPE)..."
    mkdir -p ${BUILD_DIR}
    cd ${BUILD_DIR}
    cmake -DTOOLCHAIN_PATH=${TOOLCHAIN_PATH} -DBOARD=${BOARD_TYPE} ..
    [ $? -ne 0 ] && log_error "CMake failed"

    case $1 in
        manager) make gpio_manager -j$(nproc) ;;
        test)    make test_channels -j$(nproc) ;;
        *)       make -j$(nproc) ;;
    esac
    [ $? -ne 0 ] && log_error "Build failed"
    cd ..
    log_info "Build complete"
}

# 传输
do_transfer() {
    local file=$1 name=$2
    [ ! -f "$file" ] && log_error "File not found: $file"
    log_info "Transferring $name to ${BOARD_USER}@${BOARD_IP}:${BOARD_TARGET_PATH}"
    scp -O "$file" "${BOARD_USER}@${BOARD_IP}:${BOARD_TARGET_PATH}"
    ssh "${BOARD_USER}@${BOARD_IP}" "chmod +x ${BOARD_TARGET_PATH}/${name}"
}

# 主流程
do_build "${BUILD_TARGET}"

if [ -n "${BOARD_IP}" ]; then
    case ${BUILD_TARGET} in
        manager) do_transfer "${BUILD_DIR}/gpio_manager" "gpio_manager" ;;
        test)    do_transfer "${BUILD_DIR}/test/test_channels" "test_channels" ;;
    esac

    if [ "${RUN_FLAG}" = "1" ]; then
        case ${BUILD_TARGET} in
            manager)
                ssh "${BOARD_USER}@${BOARD_IP}" "pkill -f gpio_manager" 2>/dev/null
                log_info "Running gpio_manager on board..."
                ssh -t "${BOARD_USER}@${BOARD_IP}" "${BOARD_TARGET_PATH}/gpio_manager"
                ;;
            test)
                log_info "Running test_channels on board..."
                ssh -t "${BOARD_USER}@${BOARD_IP}" "${BOARD_TARGET_PATH}/test_channels"
                ;;
        esac
    fi
fi

log_info "Done"
