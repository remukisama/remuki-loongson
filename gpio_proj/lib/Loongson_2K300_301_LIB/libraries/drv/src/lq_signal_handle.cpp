#include "lq_signal_handle.hpp"
#include "lq_common.hpp"

// 系统运行标志位
std::atomic<bool> ls_system_running(true);

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
    // 设置系统运行标志位为 false
    ls_system_running.store(false);
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

/********************************************************************************
 * @brief   信号处理函数.
 * @param   sig : 信号值.
 * @return  none.
 * @note    1. 当收到 SIGINT 信号 (Ctrl+C) 时, 打印退出信息并安全退出程序.
 *          2. 该函数会在收到 SIGINT 信号时被调用.
 ********************************************************************************/
static void __cleanup_handler(int sig)
{
    // 清理所有自动注册的资源
    lq_auto_cleanup::cleanup_all();
    const char* msg;
    switch (sig)
    {
        case SIGSEGV: msg = "\n[LQ] [\033[31mERROR\033[0m] 段错误 (SIGSEGV) → 内存访问越界/空指针; 资源已清理\n"; break;
        case SIGABRT: msg = "\n[LQ] [\033[31mERROR\033[0m] 程序异常中止 (SIGABRT) → 断言/堆损坏; 资源已清理\n"; break;
        case SIGFPE:  msg = "\n[LQ] [\033[31mERROR\033[0m] 算术错误 (SIGFPE) → 除零/数值溢出; 资源已清理\n"; break;
        case SIGBUS:  msg = "\n[LQ] [\033[31mERROR\033[0m] 总线错误 (SIGBUS) → 地址对齐错误; 资源已清理\n"; break;
        case SIGILL:  msg = "\n[LQ] [\033[31mERROR\033[0m] 非法指令 (SIGILL) → 代码损坏/不支持指令; 资源已清理\n"; break;
        case SIGSYS:  msg = "\n[LQ] [\033[31mERROR\033[0m] 非法系统调用 (SIGSYS); 资源已清理\n"; break;
        default:      msg = "\n[LQ] [\033[31mERROR\033[0m] 未知信号导致退出; 资源已清理\n"; break;
    }
    write(STDOUT_FILENO, msg, strlen(msg));
    exit(1);
}

/********************************************************************************
 * @brief   自动注册清理处理函数.
 * @param   none.
 * @return  none.
 * @note    1. 该函数会在程序启动时自动注册 __cleanup_handler 函数作为信号处理函数.
 *          2. 当收到信号时, 会调用 __cleanup_handler 函数.
 ********************************************************************************/
__attribute__((constructor(101)))
static void __lq_auto_init_cleanup(void)
{
    struct sigaction sa = {0};
    sa.sa_handler = __cleanup_handler;
    sigemptyset(&sa.sa_mask);
    // 捕获所有异常崩溃信号
    sigaction(SIGSEGV, &sa, NULL);  // 段错误
    sigaction(SIGABRT, &sa, NULL);  // abort/断言/堆损坏
    sigaction(SIGFPE,  &sa, NULL);  // 除零
    sigaction(SIGBUS,  &sa, NULL);  // 总线错误
    sigaction(SIGILL,  &sa, NULL);  // 非法指令
    sigaction(SIGSYS,  &sa, NULL);  // 非法系统调用
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
