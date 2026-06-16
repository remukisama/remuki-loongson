#include "lq_timer.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @brief   定时器无参构造函数.
 * @param   none.
 * @return  none.
 * @example lq_timer timer;
 * @note    none.
 ********************************************************************************/
lq_timer::lq_timer()
{
    this->thread_ = std::thread(&lq_timer::run, this);
}

/********************************************************************************
 * @brief   定时器析构函数.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    变量生命周期结束, 函数自动执行.
 ********************************************************************************/
lq_timer::~lq_timer()
{
    {
        std::lock_guard<std::mutex> lock(this->mutex_);
        this->running_      = false;
        this->timer_active_ = false;
        this->cv_.notify_all();         // 唤醒等待的线程
    }
    if (this->thread_.joinable())
    {
        this->thread_.join(); // 等待线程退出
    }
}

/********************************************************************************
 * @brief   设置秒级定时器.
 * @param   _s  : 秒数.
 * @param   _cb : 回调函数.
 * @return  none.
 * @example lq_timer timer;
 *          timer.set_seconds_s(1, callback_function);
 * @note    none.
 ********************************************************************************/
bool lq_timer::set_seconds_s(uint64_t _s, const timer_callback& _cb)
{
    if (_s == 0)
    {
        lq_log_error("Error: Seconds must be greater than 0!");
        return false;
    }
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->interval_ns_  = _s * 1000000000ULL;
    this->callback_     = _cb;
    this->timer_active_ = true;
    this->cv_.notify_all();
    return true;
}

/********************************************************************************
 * @brief   毫秒级定时器.
 * @param   _ms : 毫秒数.
 * @param   _cb : 回调函数.
 * @return  none.
 * @example lq_timer timer;
 *          timer.set_seconds_ms(1000, callback_function);
 * @note    none.
 ********************************************************************************/
bool lq_timer::set_seconds_ms(uint64_t _ms, const timer_callback& _cb)
{
    if (_ms == 0)
    {
        lq_log_error("Error: Milliseconds must be greater than 0!");
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    this->interval_ns_  = _ms * 1000000ULL;
    this->callback_     = _cb;
    this->timer_active_ = true;
    this->cv_.notify_all();
    return true;
}

/********************************************************************************
 * @brief   停止定时器.
 * @param   none.
 * @return  none.
 * @example lq_timer timer;
 *          timer.stop();
 * @note    none.
 ********************************************************************************/
bool lq_timer::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    this->running_ = false;
    usleep(200 * 1000);
    this->timer_active_ = false;
    this->callback_ = nullptr;
    this->cv_.notify_all();
    return true;
}

/********************************************************************************
 * @brief   运行定时器.
 * @param   none.
 * @return  none.
 * @note    内部调用.
 ********************************************************************************/
void lq_timer::run()
{
    while (1) {
        std::unique_lock<std::mutex> lock(this->mutex_);
        
        // 等待条件：退出 or 定时器激活
        this->cv_.wait(lock, [this]() {
            return !this->running_ || this->timer_active_;
        });

        if (!this->running_) break;

        uint64_t interval = this->interval_ns_;
        timer_callback cb = this->callback_;
        lock.unlock();

        auto start = std::chrono::steady_clock::now();
        try {
            if (cb) cb();
        } catch (...) {
            lq_log_error("Callback exception");
        }

        auto end = std::chrono::steady_clock::now();
        uint64_t used = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        if (used < interval) {
            timer_sleep_ns(interval - used);
        }
    }
    lq_log_info("定时器已退出!");
}

/********************************************************************************
 * @brief   休眠.
 * @param   _ns : 休眠时间.
 * @return  none.
 * @example lq_timer timer;
 *          timer.timer_sleep_ns(1000000000);
 * @note    内部函数.
 ********************************************************************************/
void lq_timer::timer_sleep_ns(uint64_t _ns)
{
    if (_ns == 0) return;
    const uint64_t threshold = 100 * 1000;
    if (_ns > threshold) {
        struct timespec req{};
        req.tv_sec = _ns / 1000000000ULL;
        req.tv_nsec = _ns % 1000000000ULL;
        clock_nanosleep(CLOCK_MONOTONIC, 0, &req, nullptr);
    } else {
        auto end = std::chrono::steady_clock::now() + std::chrono::nanoseconds(_ns);
        while (std::chrono::steady_clock::now() < end) {
            std::this_thread::yield();
        }
    }
}
