#ifndef __LQ_TIMER_HPP
#define __LQ_TIMER_HPP 

#include <iostream>
#include <cstdint>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <stdexcept>
#include <atomic>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>

// 定义回调函数类型(兼容无参数/无返回值的函数、lambda、绑定函数)
using timer_callback = std::function<void()>;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class lq_timer
{
public:
    lq_timer();                                     // 无参构造函数
    lq_timer(const lq_timer&) = delete;             // 禁止拷贝构造
    lq_timer& operator=(const lq_timer&) = delete;  // 禁止拷贝赋值
    ~lq_timer();                                    // 析构函数

public:
    bool set_seconds_s (uint64_t _s,  const timer_callback& _cb = nullptr);  // 秒级定时器
    bool set_seconds_ms(uint64_t _ms, const timer_callback& _cb = nullptr);  // 毫秒级定时器

    bool stop();                // 停止定时器
    bool is_running() const;    // 定时器是否正在运行

private:
    // bool timer_update(uint64_t _ns, const timer_callback& _cb); // 定时器更新函数
    // void timer_handler_thread();                                // 定时器处理线程函数
    void timer_sleep_ns(uint64_t _ns);                          // 纳秒级休眠函数

    void run();

private:
    mutable std::mutex      mutex_;                 // 线程安全锁
    std::condition_variable cv_;                    // 线程休眠/唤醒条件变量
    std::thread             thread_;                // 工作线程
    std::atomic<bool>       running_{true};         // 线程运行状态
    std::atomic<bool>       timer_active_{false};   // 
    uint64_t                interval_ns_{0};        // 定时器定时周期(纳秒)
    timer_callback          callback_;              // 回调函数
};

#endif
