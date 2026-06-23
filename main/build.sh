#!/bin/bash

##############################################################################
# 脚本名称：build.sh
# 脚本功能：自动查找依赖库路径 --> 配置PKG_CONFIG_PATH --> 编译项目
# 适用场景：龙芯2K300/301平台，依赖 ncnn 和 OpenCV 的项目编译
# 注意事项：需确保 LQ_Dep_libs 下存在 ncnn_install/opencv_install 目录
# 注意事项：尽量避免修改原有的文件夹结构和名称, 以免脚本无法正常运行
# 使用说明：
#       1. 增量编译：./build.sh
#       2. 增量编译 + 传输到开发板：./build.sh 192.168.1.100(写入开发板IP)
#       3. 增量编译 + 传输到开发板 + 运行：./build.sh 192.168.1.100 -r
#       4. 全量清理后编译：./build.sh --clean
#       5. 强制重新配置 CMake：./build.sh --reconfigure
##############################################################################

# ====================================================================================================================================================== #
# ============================================================== 基础配置（可根据实际情况修改）============================================================== #
# ====================================================================================================================================================== #
# 交叉编译工具链配置
TOOLCHAIN_DIR_NAME="loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"          # 工具链目录名
TOOLCHAIN_TAR_NAME="loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6.tar.xz"   # 工具链压缩包名
TOOLCHAIN_CMAKE_MACRO_FILE="./toolchain_path.cmake"                                         # 生成的 CMake 宏文件（main目录下）

# 依赖库配置
TARGET_DIR="LQ_Dep_libs"            # 目标依赖库根目录名称

DEP_LIBS="opencv_install ncnn_install ffmpeg_install"   # 配置需要检测的依赖库列表
PKG_REL_PATH="lib/pkgconfig/"                           # 所以库统一的pkgconfig路径
# 各库pkg-config检测名
PKG_CONFIG_NAMES="\
    opencv4 \
    ncnn \
    libavformat \
    libavcodec \
    libavdevice \
    libavutil \
"

# 路径配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"                    # 压缩包/文件夹的目录路径（main 同级目录）
TOOLS_DIR="${PARENT_DIR}/tools"                                 # tools 目录路径（main 同级目录）
SEARCH_DIR="${SEARCH_DIR:-${PARENT_DIR}}"                       # 搜索根路径
OPENCV_PKG_REL_PATH="opencv_install/lib/pkgconfig/"             # OpenCV pkgconfig相对路径
NCNN_PKG_REL_PATH="ncnn_install/lib/pkgconfig/"                 # ncnn pkgconfig相对路径

# 编译参数配置
MAX_DEPTH=5                 # 搜索最大深度
BUILD_THREADS=$(nproc)      # 编译线程数
BUILD_DIR="build"           # 构建目录名称

# SCP 传输配置
BOARD_USER="root"                                           # 登录开发板的用户名
BOARD_TARGET_PATH="${BOARD_TARGET_PATH:-~}"                  # 开发板目标路径
EXECUTABLE_NAME="main"                                      # 可执行程序名（根据实际情况修改）
REMOTE_EXEC_CMD="${BOARD_TARGET_PATH}/${EXECUTABLE_NAME}"   # 远程执行命令
BOARD_PHYSICAL_TTY="/dev/console"                           # 开发板物理终端（指向开发板主屏幕）
BOARD_LOG_FILE="${BOARD_LOG_FILE:-${EXECUTABLE_NAME}.log}"   # 程序日志文件（开发板上保存输出，方便排查）

# ====================================================================================================================================================== #
# ===================================================================== 核心函数定义 ===================================================================== #
# ====================================================================================================================================================== #
##################################################
# 函数作用：普通日志输出函数（绿色打印）
# 参数说明：
#       $1 : 日志内容
# 使用示例：log_info "这是一个普通日志"
##################################################
function log_info() {
    echo -e "\033[32m[$(date +%Y-%m-%d\ %H:%M:%S)] [INFO ] $1\033[0m"
}
##################################################
# 函数作用：警告日志输出函数（黄色打印）
# 参数说明：
#       $1 : 日志内容
# 使用示例：log_warn "这是一个警告日志"
##################################################
function log_warn() {
    echo -e "\033[33m[$(date +%Y-%m-%d\ %H:%M:%S)] [WARN ] $1\033[0m"
}
##################################################
# 函数作用：错误日志输出函数（红色打印）
# 参数说明：
#       $1 : 日志内容
# 使用示例：log_error "这是一个错误日志"
##################################################
function log_error() {
    echo -e "\033[31m[$(date +%Y-%m-%d\ %H:%M:%S)] [ERROR] $1\033[0m"
    exit 1
}

# 自动检测并安装pkg-config
function check_and_install_pkgconfig() {
    log_info "=================================================================== 检测各依赖工具 ==================================================================="
    # 检测pkg-config工具是否安装
    if command -v pkg-config &>/dev/null; then
        local pkg_version=$(pkg-config --version)
        log_info "✅ 已安装 pkg-config，版本：${pkg_version}"
        log_info "======================================================================================================================================================\n"
        return 0
    fi
    # 未安装，提示并开始安装
    log_warn "🔍 未检测到 pkg-config 工具，开始自动安装..."
    log_info "🔧 正在尝试更新软件源..."
    # 更新软件源
    if ! sudo apt update -y 2>&1; then
        log_error "❌ 更新软件源失败，请检查网络连接或手动执行 sudo apt update"
    fi
    log_info "✅ 软件源更新成功"
    log_info "🔧 正在尝试安装 pkg-config 工具..."
    # 安装pkg-config
    if ! sudo apt install -y pkg-config 2>&1; then
        log_error "❌ 自动安装 pkg-config 工具失败，请手动执行 sudo apt install -y pkg-config"
    fi
    # 检查安装结果
    if command -v pkg-config &>/dev/null; then
        log_info "✅ pkg-config 安装成功，版本：$(pkg-config --version)"
        log_info "======================================================================================================================================================\n"
    else
        log_error "❌ pkg-config 安装后仍未检测到，请检查软件源配置或手动安装"
    fi
}

##################################################
# 函数作用：检测并安装依赖库
# 参数说明：
#       $1 : 任意数量的依赖目录
# 使用说明：内部使用，不用管，想要添加依赖库直接在
#         REQUIRED_DEPS 变量中添加即可
##################################################
function check_and_install_deps() {
    local missing_deps=()   # 存储未找到的依赖库
    local dep_list=("$@")   # 依赖库列表
    # 逐个检测依赖，收集缺失项
    log_info "=================================================================== 检测各依赖工具 ==================================================================="
    log_info "🔍 开始检测依赖（共${#dep_list[@]}个）"
    for dep in "${dep_list[@]}"; do
        if command -v "${dep}" &>/dev/null; then
            log_info "✅ 已安装 ${dep}    \t版本可输入 ${dep} --version 验证版本号"
        else
            log_warn "⚠️ 未安装 ${dep}！"
            missing_deps+=("${dep}")
        fi
    done
    # 统一更新+安装缺失的依赖
    if [[ ${#missing_deps[@]} -eq 0 ]]; then
        log_info "✅ 所有依赖库都已安装，无需更新"
        log_info "======================================================================================================================================================\n"
        return 0
    fi
    log_info "🔧 开始自动安装确实依赖库（共${#missing_deps[@]}个）"
    log_info "🔧 尝试更新软件源(sudo apt update)..."
    if ! sudo apt update -y 2>&1; then
        log_error "❌ 升级软件源失败，请检查网络连接或手动执行 sudo apt update"
    fi
    log_info "✅ 软件源更新成功\n"
    for dep in "${missing_deps[@]}"; do 
        log_info "🔧 尝试安装 ${dep}..."
        if sudo apt install -y "${dep}" 2>&1; then
            if command -v "${dep}" &>/dev/null; then
                log_info "✅ ${dep} 安装成功"
            else
                log_error "❌ ${dep} 安装后检测，但未找到，请检查软件源配置"
            fi
        else
            log_error "❌ ${dep} 安装失败，请手动执行 sudo apt install -y ${dep}"
        fi
    done
    log_info "✅ 所有依赖库都已安装"
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
# 函数作用：通用 PKG_CONFIG_PATH 配置函数
# 参数说明：无
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
    for lib_dir_name in $(echo ${DEP_LIBS} | tr ' ' '\n'); do
        # 按索引取对应的检测名
        pkg_check_name=$(echo ${PKG_CONFIG_NAMES} | cut -d' ' -f$((i+1)))
        
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
    for lib_dir_name in ${PKG_CONFIG_NAMES}; do
        if pkg-config --exists "${lib_dir_name}" 2>/dev/null; then
            log_info "✅ pkg-config 验证成功：${lib_dir_name}"
            log_info "  ├─ 编译参数：$(pkg-config --cflags ${lib_dir_name})"
            log_info "  └─ 链接参数：$(pkg-config --libs ${lib_dir_name})\n"
        else
            log_warn "⚠️ pkg-config 未找到 ${lib_dir_name} 库（非致命错误，继续执行）"
            # 改为警告而非终止，避免单个库问题导致脚本中断
        fi
    done
    log_info "🎉 PKG_CONFIG_PATH 配置 & 验证完毕!"
    log_info "======================================================================================================================================================\n"
}

##################################################
# 函数作用：IP 地址合法性校验
# 参数说明：IP 地址
# 使用说明：内部使用，不用管
##################################################
function is_valid_ip() {
    local ip=$1
    # 正则匹配IPv4地址（简单且实用的校验规则）
    local ip_regex="^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$"
    [[ $ip =~ $ip_regex ]] && return 0 || return 1  # 0-->合法IP，1-->非法IP
}

##################################################
# 函数作用：SCP 传输可执行程序到开发板
# 参数说明：
#       $1 : 开发板 IP 地址
# 使用说明：内部使用，不用管
##################################################
function scp_to_board() {
    local board_ip=$1
    local local_exec_path="${BUILD_DIR}/${EXECUTABLE_NAME}"
    # 检查可执行程序是否存在
    if [ ! -f "${local_exec_path}" ]; then
        log_error "❌ 可执行程序不存在！路径：${local_exec_path}\n请确认编译是否成功，或修改脚本中 EXECUTABLE_NAME 配置！"
    fi
    # 测试开发板连通性
    log_info "🔧 测试与开发板 ${board_ip} 的连通性..."
    if ! ping -c 2 -W 3 "${board_ip}" >/dev/null 2>&1; then
        log_warn "⚠️ 开发板 ${board_ip} 无法 ping 通！尝试直接传输..."
    fi
    # 执行SCP传输
    log_info "🔧 开始传输可执行程序到开发板 ${board_ip}:${BOARD_TARGET_PATH}"
    if scp -O -o ConnectTimeout=10 "${local_exec_path}" "${BOARD_USER}@${board_ip}:${BOARD_TARGET_PATH}"; then
        log_info "✅ 传输成功！开发板路径：${BOARD_USER}@${board_ip}:${BOARD_TARGET_PATH}/${EXECUTABLE_NAME}"
        # 传输后自动添加执行权限
        ssh -o ConnectTimeout=10 "${BOARD_USER}@${board_ip}" "chmod +x ${REMOTE_EXEC_CMD}" >/dev/null 2>&1
        log_info "✅ 已为程序添加执行权限"
    else
        log_error "❌ 传输失败！请检查：
        1. 开发板 IP 是否正确
        2. 开发板是否开启 SSH 服务
        3. 开发板 ${BOARD_TARGET_PATH} 路径是否有写入权限
        4. PC与开发板是否在同一网络"
    fi
}

##################################################
# 函数作用：停止开发板上的旧程序实例（传输前执行）
# 参数说明：
#       $1 : 开发板 IP 地址
# 使用说明：内部使用，不用管
##################################################
function stop_remote_program() {
    local board_ip=$1
    log_info "================================================================== 停止开发板旧程序 =================================================================="
    # 检查是否有运行中的程序
    local pid=$(ssh -o ConnectTimeout=10 "${BOARD_USER}@${board_ip}" "pgrep -f ${EXECUTABLE_NAME}")
    if [ -z "${pid}" ]; then
        log_warn "⚠️ 未检测到运行中的 ${EXECUTABLE_NAME} 程序，跳过停止步骤"
        return
    fi
    # 停止程序（强制杀死）
    log_info "🔍 检测到运行中的程序，PID：${pid}，开始停止..."
    ssh -o ConnectTimeout=10 "${BOARD_USER}@${board_ip}" "kill -9 ${pid} >/dev/null 2>&1"
    # 验证是否停止成功
    sleep 1
    local pid_after=$(ssh -o ConnectTimeout=10 "${BOARD_USER}@${board_ip}" "pgrep -f ${EXECUTABLE_NAME}")
    if [ -z "${pid_after}" ]; then
        log_info "✅ 程序已成功停止（PID：${pid}）"
    else
        log_warn "⚠️ 程序停止失败，剩余PID：${pid_after}（可能需要手动停止）"
    fi
    log_info "======================================================================================================================================================\n"
}

##################################################
# 函数作用：远程执行程序
# 参数说明：
#       $1 : 开发板 IP 地址
# 使用说明：内部使用，不用管
##################################################
function run_remote_program() {
    local board_ip=$1
    log_info "==================== 远程执行程序 ===================="
    log_info "🔧 开始在开发板 ${board_ip} 执行程序：${REMOTE_EXEC_CMD}"
    # 创建远程执行的临时脚本（去掉ldd检查）
    local temp_script="${BOARD_TARGET_PATH}/run_app.sh"
    local remote_script_content="
        #!/bin/bash
        set -e  # 出错立即退出
        echo '===== 开始执行程序 =====' >> ${BOARD_LOG_FILE}
        echo '程序路径：${REMOTE_EXEC_CMD}' >> ${BOARD_LOG_FILE}
        
        # 停止旧实例
        pkill -f ${EXECUTABLE_NAME} >/dev/null 2>&1 || true
        echo '已停止旧程序实例' >> ${BOARD_LOG_FILE}
        
        # 检查程序是否存在
        if [ ! -f ${REMOTE_EXEC_CMD} ]; then
            echo '错误：程序不存在！' >> ${BOARD_LOG_FILE}
            exit 1
        fi
        
        # 检查执行权限
        if [ ! -x ${REMOTE_EXEC_CMD} ]; then
            echo '添加执行权限' >> ${BOARD_LOG_FILE}
            chmod +x ${REMOTE_EXEC_CMD}
        fi
        
        # 注释掉ldd检查（开发板无ldd命令）
        # echo '程序依赖库：' >> ${BOARD_LOG_FILE}
        # ldd ${REMOTE_EXEC_CMD} >> ${BOARD_LOG_FILE} 2>&1
        
        # 执行程序（禁用缓冲+后台运行+输出到屏幕+日志）
        echo '启动程序...' >> ${BOARD_LOG_FILE}
        stdbuf -o0 -e0 nohup ${REMOTE_EXEC_CMD} > ${BOARD_PHYSICAL_TTY} 2>&1 >> ${BOARD_LOG_FILE} &
        
        # 等待程序启动，输出PID
        sleep 1
        PID=\$(pgrep -f '${EXECUTABLE_NAME}')
        if [ -n \"\$PID\" ]; then
            echo \"程序启动成功！PID：\$PID\" >> ${BOARD_LOG_FILE}
            exit 0
        else
            echo '程序启动失败！无PID' >> ${BOARD_LOG_FILE}
            exit 1
        fi
    "
    # 上传并执行临时脚本
    log_info "🔧 上传执行脚本到开发板..."
    echo "${remote_script_content}" | ssh -o ConnectTimeout=10 "${BOARD_USER}@${board_ip}" "cat > ${temp_script} && chmod +x ${temp_script}"
    if [ $? -ne 0 ]; then
        log_error "❌ 上传临时脚本失败！请检查开发板网络"
    fi
    log_info "🔧 执行启动脚本，日志写入：${board_ip}:${BOARD_LOG_FILE}"
    if ssh -o ConnectTimeout=20 "${BOARD_USER}@${board_ip}" "bash ${temp_script}"; then
        log_info "✅ 程序启动脚本执行成功！"
        log_info "  ├─ 开发板程序PID：$(ssh ${BOARD_USER}@${board_ip} "pgrep -f ${EXECUTABLE_NAME}")"
        log_info "  ├─ 查看运行日志：ssh ${BOARD_USER}@${board_ip} 'cat ${BOARD_LOG_FILE}'"
        log_info "  ├─ 停止程序：1.在当前编译终端运行 ssh ${BOARD_USER}@${board_ip} 'pkill -f ${EXECUTABLE_NAME}'"
        log_info "  └─ 停止程序：2.在开发板终端运行 pkill -f ${EXECUTABLE_NAME}"
    else
        log_error "❌ 程序启动失败！请查看开发板日志：
        ssh ${BOARD_USER}@${board_ip} 'cat ${BOARD_LOG_FILE}'"
    fi
    # 清理临时脚本
    ssh "${BOARD_USER}@${board_ip}" "rm -f ${temp_script}" >/dev/null 2>&1
}

# ====================================================================================================================================================== #
# ==================================================================== 解析传入参数 ====================================================================== #
# ====================================================================================================================================================== #
# 获取传入参数
CLEAN_BUILD=false
FORCE_RECONFIGURE=false
RUN_PROGRAM=false       # 默认不运行
BOARD_IP=""             # 开发板的 IP

while [ $# -gt 0 ]; do
    case "$1" in
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        --reconfigure)
            FORCE_RECONFIGURE=true
            shift
            ;;
        -r)
            RUN_PROGRAM=true
            shift
            ;;
        --help|-h)
            echo "用法："
            echo "  ./build.sh                         # 增量编译"
            echo "  ./build.sh 192.168.1.100           # 增量编译 + 传输"
            echo "  ./build.sh 192.168.1.100 -r        # 增量编译 + 传输 + 运行"
            echo "  ./build.sh --clean                 # 删除 build 后全量编译"
            echo "  ./build.sh --reconfigure           # 强制重新执行 CMake 配置"
            exit 0
            ;;
        *)
            if [ -z "${BOARD_IP:-}" ]; then
                BOARD_IP="$1"
                shift
            else
                log_error "❌ 未知参数：$1"
            fi
            ;;
    esac
done

BOARD_IP="${BOARD_IP:-}"       # 开发板的 IP

# 参数校验逻辑
if [ -n "${BOARD_IP}" ]; then
    # 校验IP合法性
    if ! is_valid_ip "${BOARD_IP}"; then
        log_error "❌ 参数不是合法IP地址！用法：\n  增量编译：./build.sh\n  增量编译+传输：./build.sh 192.168.1.100\n  增量编译+传输+运行：./build.sh 192.168.1.100 -r\n  全量编译：./build.sh --clean\n  重新配置：./build.sh --reconfigure"
    fi
    if [ "${RUN_PROGRAM}" = true ]; then
        log_info "🔧 本次执行：编译 → 传输 → 远程运行程序\n"
    else
        log_info "🔧 本次执行：编译 → 传输（未指定-r，不运行程序）\n"
    fi
else
    if [ "${RUN_PROGRAM}" = true ]; then
        log_warn "⚠️ 未传入开发板 IP，-r 参数无效，将仅编译\n"
        RUN_PROGRAM=false
    fi
    log_info "🔧 本次执行：仅编译（未传入IP，跳过传输/运行）\n"
fi

if [ "${CLEAN_BUILD}" = true ]; then
    log_warn "⚠️ 已启用 --clean：本次会删除 ${BUILD_DIR} 后全量编译"
fi

if [ "${FORCE_RECONFIGURE}" = true ]; then
    log_warn "⚠️ 已启用 --reconfigure：本次会强制重新执行 CMake 配置"
fi

# 该指令表示任意指令执行失败，立即终止脚本
set -e

# 打印关键路径
log_info "================================================================= 调试信息：关键路径 ================================================================="
log_info "脚本所在目录 ---> SCRIPT_DIR：${SCRIPT_DIR}"
log_info "上级所在目录 ---> PARENT_DIR：${PARENT_DIR}"
log_info "工具所在目录 ---> TOOLS_DIR ：${TOOLS_DIR}"
log_info "依赖库所在目录 -> TARGET_DIR：${TARGET_DIR}"
log_info "依赖库完整路径 -> target_dir_full：${TOOLS_DIR}/${TARGET_DIR}"
log_info "依赖库的列表 ---> DEP_LIBS  ：${DEP_LIBS}"
log_info "======================================================================================================================================================\n"

# 脚本执行路径切换到脚本所在目录
cd "${SCRIPT_DIR}"
log_info "🔧 脚本开始执行，当前工作目录：$(pwd)\n"

# 先检查 tools 目录是否存在
if [ ! -d "${TOOLS_DIR}" ]; then
    log_error "❌ tools 目录不存在！预期路经：${TOOLS_DIR}，请确认目录结构！"
fi

# 检测并安装 pkg-config（函数）
# check_and_install_pkgconfig
function main() {
    # 定义编译工程需要用到的依赖列表
    local REQUIRED_DEPS=(
        "pkg-config"
        "cmake"
    )
    check_and_install_deps "${REQUIRED_DEPS[@]}"
}

main

# 配置交叉编译工具链
log_info "================================================================= 处理交叉编译工具链 ================================================================="
# 拼接工具链路径
toolchain_dir_full="${TOOLS_DIR}/${TOOLCHAIN_DIR_NAME}"
toolchain_tar_full="${TOOLS_DIR}/${TOOLCHAIN_TAR_NAME}"

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
        tar -xvf "${toolchain_tar_full}" -C "${TOOLS_DIR}" || log_error "❌ 解压交叉编译工具链压缩包失败！"
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

# 生成 CMake 可识别的宏文件
log_info "================================================================== CMake 宏文件生成 =================================================================="
log_info "🔧 检查 CMake 宏文件：${TOOLCHAIN_CMAKE_MACRO_FILE}"
TOOLCHAIN_CMAKE_CONTENT="set(CMAKE_TOOLCHAIN_PATH \"\${CMAKE_CURRENT_LIST_DIR}/../tools/${TOOLCHAIN_DIR_NAME}\" CACHE PATH \"Loongson toolchain path\" FORCE)"
if [ ! -f "${TOOLCHAIN_CMAKE_MACRO_FILE}" ] || ! grep -Fxq "${TOOLCHAIN_CMAKE_CONTENT}" "${TOOLCHAIN_CMAKE_MACRO_FILE}"; then
    printf '%s\n' "${TOOLCHAIN_CMAKE_CONTENT}" > "${TOOLCHAIN_CMAKE_MACRO_FILE}"
    log_info "✅ CMake 宏文件已更新"
else
    log_info "✅ CMake 宏文件内容未变化，跳过写入"
fi
# 验证宏文件是否生成
if [ -f "${TOOLCHAIN_CMAKE_MACRO_FILE}" ]; then
    log_info "✅ CMake 宏文件检查完成！内容："
    cat "${TOOLCHAIN_CMAKE_MACRO_FILE}" | grep -v "^#" | grep -v "^$"
else
    log_error "❌ CMake 宏文件生成失败！"
fi
log_info "======================================================================================================================================================\n"

# 处理常驻依赖库目录
log_info "================================================================= 处理常驻依赖库目录 ================================================================="
target_dir_full="${TOOLS_DIR}/${TARGET_DIR}"
log_info "🔍 检测常驻依赖库目录：${target_dir_full}"
# 检查LQ_Dep_libs常驻目录是否存在
if [ ! -d "${target_dir_full}" ]; then
    log_error "❌ 常驻依赖库目录 ${TARGET_DIR} 不存在！
    预期路径：${target_dir_full}
    请先创建该目录：mkdir -p ${target_dir_full}"
fi
log_info "✅ 找到常驻依赖库目录：${target_dir_full}"
log_info "======================================================================================================================================================\n"

# 遍历所有依赖库（纯路径赋值，无日志污染）
log_info "=================================================================== 遍历处理依赖库 ==================================================================="
current_idx=0
total_libs=$(echo ${DEP_LIBS} | tr ' ' '\n' | wc -l)
for lib_dir_name in $(echo ${DEP_LIBS} | tr ' ' '\n'); do
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

# 配置 PKG_CONFIG_PATH
setup_pkgconfig_path

# 构建编译项目
log_info "====================================================================== 编译项目 ======================================================================"
if [ "${CLEAN_BUILD}" = true ] && [ -d "${BUILD_DIR}" ]; then
    log_warn "⚠️ 执行全量清理构建：删除 ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
fi

if [ ! -f "${BUILD_DIR}/CMakeCache.txt" ] || [ "${FORCE_RECONFIGURE}" = true ]; then
    log_info "🔧 执行 CMake 配置：${BUILD_DIR}"
    cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" || log_error "❌ cmake 配置失败！"
else
    log_info "✅ 检测到已有 CMake 构建缓存，使用增量构建"
fi

log_info "🔧 开始增量编译项目（线程数：${BUILD_THREADS}）"
cmake --build "${BUILD_DIR}" --parallel "${BUILD_THREADS}" || log_error "❌ 编译失败！"
log_info "✅ 项目编译完成！未修改的源文件不会重复编译。"
log_info "======================================================================================================================================================\n"

# SCP 传输到开发板
if [ -n "${BOARD_IP}" ]; then
    LOCAL_EXEC_PATH="${BUILD_DIR}/${EXECUTABLE_NAME}"
    if [ ! -f "${LOCAL_EXEC_PATH}" ]; then
        log_error "❌ 可执行程序不存在！路径：${LOCAL_EXEC_PATH}"
    fi

    LOCAL_SHA="$(sha256sum "${LOCAL_EXEC_PATH}" | awk '{print $1}')"
    REMOTE_SHA="$(ssh -o ConnectTimeout=5 "${BOARD_USER}@${BOARD_IP}" "sha256sum '${REMOTE_EXEC_CMD}' 2>/dev/null | awk '{print \$1}'" 2>/dev/null || true)"

    if [ -n "${REMOTE_SHA}" ] && [ "${LOCAL_SHA}" = "${REMOTE_SHA}" ]; then
        log_info "✅ 开发板上的可执行文件与本地一致，跳过停止和传输"
        if [ "${RUN_PROGRAM}" = true ]; then
            run_remote_program "${BOARD_IP}" # （函数）
        fi
    else
        stop_remote_program "${BOARD_IP}"
        log_info "==================================================================== 传输到开发板 ===================================================================="
        scp_to_board "${BOARD_IP}" # （函数）

        # 如果指定了 -r，远程运行程序
        if [ "${RUN_PROGRAM}" = true ]; then
            run_remote_program "${BOARD_IP}" # （函数）
        fi
    fi
    log_info "======================================================================================================================================================\n"
fi

# 脚本执行完成
log_info "🎉 脚本执行完成！"
log_info "🔍 可执行程序路径：$(pwd)/${BUILD_DIR}/${EXECUTABLE_NAME}"

exit 0
