#!/bin/bash

BOARD_PHYSICAL_TTY="/dev/console"               # 开发板物理终端（指向开发板主屏幕）
BOARD_LOG_FILE="/tmp/${EXECUTABLE_NAME}.log"    # 程序日志文件（开发板上保存输出，方便排查）

# SCP 传输配置
BOARD_USER="root"                               # 登录开发板的用户名
BOARD_TARGET_PATH="/home/root"                  # 开发板目标路径

##################################################
# 函数作用：SCP 传输可执行程序到开发板
# 参数说明：
#       $1 : 开发板 IP 地址
#       $2 : 本地可执行程序所在目录
# 使用说明：内部使用，不用管
##################################################
function scp_to_board() {
    local board_ip=$1
    local local_exec_path="$2/$3"
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
        log_info "✅ 传输成功！开发板路径：${BOARD_USER}@${board_ip}:${BOARD_TARGET_PATH}/$3"
        # 传输后自动添加执行权限
        ssh -o ConnectTimeout=10 "${BOARD_USER}@${board_ip}" "chmod +x ${BOARD_TARGET_PATH}/$3" >/dev/null 2>&1
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
    local REMOTE_EXEC_CMD="${BOARD_TARGET_PATH}/${EXECUTABLE_NAME}"
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

##################################################
# 函数作用：SCP 传输到开发板
# 参数说明：
#       $1 : 开发板 IP 地址
#       $2 : 本地可执行程序所在目录
#       $3 : 可执行程序名（根据实际情况修改）
# 使用说明：内部使用，不用管
##################################################
function scp_transfer_dev_board() {
    if is_valid_ip "$1"; then
        stop_remote_program $1
        log_info "==================================================================== 传输到开发板 ===================================================================="
        scp_to_board $1 $2 $3 # （函数）

        # 如果指定了 -r，远程运行程序
        if [ "${RUN_PROGRAM}" = true ]; then
            run_remote_program $1 # （函数）
        fi
        log_info "======================================================================================================================================================\n"
    fi
}
