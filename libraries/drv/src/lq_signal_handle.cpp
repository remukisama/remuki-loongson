#include "lq_signal_handle.hpp"

extern "C" {

// 自定义退出处理函数指针
static lq_empty_cb_t g_exit_cb = nullptr;

/********************************************************************************
 * @brief   信号处理函数.
 * @param   sig : 信号值.
 * @return  none.
 * @note    1. 当收到 SIGINT 信号 (Ctrl+C) 时, 打印退出信息并安全退出程序.
 *          2. 该函数会在收到 SIGINT 信号时被调用.
 ********************************************************************************/
static void __sigint_handler(int sig)
{
    (void)sig;
    // 调用自定义退出处理函数
    if (g_exit_cb != nullptr) {
        g_exit_cb();
    }
    const char msg[] = "\n[LQ] [\033[32mEXIT\033[0m ] Ctrl+C 安全退出\n";
    write(STDOUT_FILENO, msg, sizeof(msg));
    exit(0);
}

/********************************************************************************
 * @brief   自动注册信号处理函数.
 * @param   none.
 * @return  none.
 * @note    1. 该函数会在程序启动时自动注册 __sigint_handler 函数作为 SIGINT 信号的处理函数.
 *          2. 当收到 SIGINT 信号 (Ctrl+C) 时, 会调用 __sigint_handler 函数.
 ********************************************************************************/
__attribute__((constructor))
static void __lq_auto_setup_signal(void)
{
    struct sigaction sa = {0};
    sa.sa_handler = __sigint_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
}

}

/********************************************************************************
 * @brief   设置自定义 SIGINT 信号处理函数.
 * @param   cb : 信号处理函数指针.
 * @return  none.
 * @note    1. 该函数会将 cb 函数指针赋值给 g_sigint_cb 变量.
 *          2. 当收到 SIGINT 信号 (Ctrl+C) 时, 会调用 cb 函数.
 ********************************************************************************/
void lq_signal_set_exit_cb(lq_empty_cb_t cb)
{
    g_exit_cb = cb;
}
