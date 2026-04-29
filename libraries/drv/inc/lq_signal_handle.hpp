#ifndef __LQ_SIGNAL_HANDLE_HPP
#define __LQ_SIGNAL_HANDLE_HPP

#include <signal.h>
#include <unistd.h>
#include <cstdlib>
#include "lq_common.hpp"

/********************************************************************************
 * @brief   设置自定义 SIGINT 信号处理函数.
 * @param   cb : 信号处理函数指针.
 * @return  none.
 * @note    1. 该函数会将 cb 函数指针赋值给 g_exit_cb 变量.
 *          2. 当收到 SIGINT 信号 (Ctrl+C) 时, 会调用 cb 函数.
 ********************************************************************************/
void lq_signal_set_exit_cb(lq_empty_cb_t cb);

#endif
