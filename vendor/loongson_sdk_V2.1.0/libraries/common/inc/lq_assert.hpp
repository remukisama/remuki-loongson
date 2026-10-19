#ifndef __LQ_ASSERT_HPP
#define __LQ_ASSERT_HPP

/********************************************************************************
 * @brief   适配 Linux 开发板的断言与日志打印模块
 * @note    支持：编译期错误终止 + 运行期分级日志 + 文件/函数/行号/PID定位
 ********************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <limits.h>

/* ==================== 辅助宏（内部使用，加下划线区分） ==================== */
// 辅助宏：将数字转为字符串（用于行号拼接）
#define _lq_stringify(x)                #x
#define _lq_to_string(x)                _lq_stringify(x)

/* ==================== 编译期强制检查 ==================== */
// 编译器断言宏（支持自定义提示，编译失败时显示信息）
#define lq_compile_assert(expr, msg)    static_assert(expr, "[" __FILE__ ":" _lq_to_string(__LINE__) "] " msg)

// 编译期错误终止宏（强制停止编译，打印文件 + 行号 + 自定义信息）
#define lq_compile_error(msg)           _Pragma(_lq_to_string(GCC error "[" __FILE__ ":" _lq_to_string(__LINE__) "] compile error: " msg))

/* ==================== 配置区 ==================== */
// 日志总开关：1-启用，0-禁用（编译期控制，0 时所有日志宏为空操作）
#define LQ_ENABLE_ASSERT_LOG            ( 1 )

// 日志级别控制：按需开启/关闭对应级别(仅打印不终止)
#define LQ_LOG_LEVEL_INFO               ( 1 )
#define LQ_LOG_LEVEL_WARN               ( 1 )
#define LQ_LOG_LEVEL_ERROR              ( 1 )

// 断言开关：1-启用（error级会触发断言），0-禁用（仅打印不终止）
#define LQ_ENABLE_ASSERT                ( 1 )

// 日志输出方式：1-标准输出（stdout），2-系统日志（syslog），3-指定文件（仅1/2/3合法）
#define LQ_LOG_OUTPUT_MODE              ( 1 )

// 若选择文件输出，指定日志文件路径（Linux开发板路径）
#define LQ_LOG_FILE_PATH                ( "/tmp/app_log.txt" )

// 最大日志缓冲区大小（必须 > 0）
#define LQ_LOG_BUF_SIZE                 ( 512 )

/* ==================== 编译期合法性检查 ==================== */
// 检查 1：日志级别只能是 0 或 1
lq_compile_assert((LQ_LOG_LEVEL_INFO  == 0 || LQ_LOG_LEVEL_INFO  == 1), "LQ_LOG_LEVEL_INFO must be 0 or 1");
lq_compile_assert((LQ_LOG_LEVEL_WARN  == 0 || LQ_LOG_LEVEL_WARN  == 1), "LQ_LOG_LEVEL_WARN must be 0 or 1");
lq_compile_assert((LQ_LOG_LEVEL_ERROR == 0 || LQ_LOG_LEVEL_ERROR == 1), "LQ_LOG_LEVEL_ERROR must be 0 or 1");

// 检查 2：断言开关只能是 0 或 1
lq_compile_assert((LQ_ENABLE_ASSERT == 0 || LQ_ENABLE_ASSERT == 1), "LQ_ENABLE_ASSERT must be 0 or 1");

// 检查 3：输出模式只能是 1/2/3
lq_compile_assert((LQ_LOG_OUTPUT_MODE >= 1 && LQ_LOG_OUTPUT_MODE <= 3), "LQ_LOG_OUTPUT_MODE must be 1, 2, or 3");

// 检查 4：缓冲区大小是否合法
lq_compile_assert((LQ_LOG_BUF_SIZE >0 && LQ_LOG_BUF_SIZE <= 1024), "LQ_LOG_BUF_SIZE must be between 1 and 1024");

// 检查 5：Linux 系统环境
#if !defined(__linux__)
lq_compile_assert("This program only supports Linux platform!");
#endif

// 检查 6：龙芯架构检查
#if !defined(__loongarch__)
lq_compile_assert("This program only supports LoongArch (Loongson) platform!");
#endif

// 检查 7：文件输出模式时路径非空
#if LQ_LOG_OUTPUT_MODE == 3
lq_compile_assert((LQ_LOG_FILE_PATH[0] != '\0'), "LQ_LOG_FILE_PATH cannot be empty when LQ_LOG_OUTPUT_MODE=3");
#endif

/********************************************************************************
 * @brief   提取文件名（去掉路径）.
 * @param   _path : 文件路径.
 * @return  去掉路径的文件名.
 ********************************************************************************/
static const char* lq_get_filename(const char* _path)
{
    // 空指针保护
    if (_path == NULL || *_path == '\0') {
        return "unknow_file";
    }

    const char* filename = strrchr(_path, '/');
    return (filename != NULL) ? (filename + 1) : _path;
}

/********************************************************************************
 * @brief   Linux 平台格式化输出函数.
 * @param   _fmt : 输出内容.
 * @param   ...  : 可变参数.
 * @return  none.
 ********************************************************************************/
static void lq_linux_printf(const char *_fmt, ...)
{
    // 空指针保护
    if (_fmt == NULL) {
        return;
    }

    va_list args;
    va_start(args, _fmt);

#if LQ_LOG_OUTPUT_MODE == 1
    // 标准输出
    vprintf(_fmt, args);
#elif LQ_LOG_OUTPUT_MODE == 2
    // 系统日志
    vsyslog(LOG_INFO, _fmt, args);
#elif LQ_LOG_OUTPUT_MODE == 3
    // 文件输出
    int fd = open(LQ_LOG_FILE_PATH, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd >= 0) {
        char buf[LQ_LOG_BUF_SIZE] = {0};
        int len = vsnprintf(buf, sizeof(buf)-1, _fmt, args);
        len = (len < 0) ? 0 : (len >= (int)sizeof(buf)) ? ((int)sizeof(buf) - 1) : len;
        write(fd, buf, (size_t)len);
        close(fd);
    }
#endif

    va_end(args);
}

/* ==================== 核心宏定义 ==================== */
#if LQ_ENABLE_ASSERT_LOG

// 基础日志宏
#define _lq_base_log(level, fmt, ...)           do { \
                                                    lq_linux_printf("[LQ] [%s] [PID:%d] <%s:%s:%d> " fmt "\n", \
                                                        level, (int)getpid(), lq_get_filename(__FILE__), __func__, __LINE__, ##__VA_ARGS__); \
                                                } while (0)

// 分级日志宏
#if LQ_LOG_LEVEL_INFO
#define lq_log_info(fmt, ...)                   _lq_base_log("INFO ", fmt, ##__VA_ARGS__)
#else
#define lq_log_info(fmt, ...)                   do {} while (0)
#endif

#if LQ_LOG_LEVEL_WARN
#define lq_log_warn(fmt, ...)                   _lq_base_log("WARN ", fmt, ##__VA_ARGS__)
#else
#define lq_log_warn(fmt, ...)                   do {} while (0)
#endif

#if LQ_LOG_LEVEL_ERROR
#define lq_log_error(fmt, ...)                  _lq_base_log("ERROR", fmt, ##__VA_ARGS__)
#else
#define lq_log_error(fmt, ...)                  do {} while (0)
#endif

// 断言宏
#if LQ_ENABLE_ASSERT
#define lq_assert(expr)                         do { \
                                                    if (!(expr)) { \
                                                        lq_log_error("ASSERT FAILED: %s", #expr); \
                                                        assert(expr);   /* Linux 下终止进程，生成core dump */ \
                                                    } \
                                                } while (0)
#else
#define lq_assert(expr)                         do { \
                                                    if (!(expr)) { \
                                                        lq_log_error("ASSERT FAILED: %s", #expr); \
                                                    } \
                                                } while (0)
#endif

#else
// 禁用日志时的空宏
#define lq_log_info(fmt, ...)                   do {} while (0)
#define lq_log_warn(fmt, ...)                   do {} while (0)
#define lq_log_error(fmt, ...)                  do {} while (0)
#define lq_assert(expr)                         do {} while (0)

#endif

#endif
