#ifndef GPIO_PROTECT_HEARTBEAT_HPP_
#define GPIO_PROTECT_HEARTBEAT_HPP_

void gpio_protect_heartbeat_init(void);
void gpio_protect_heartbeat_pulse(void);
void gpio_protect_heartbeat_deinit(void);
bool gpio_protect_heartbeat_require_ready(int timeout_ms);
// 检查 watchdog 进程是否仍在更新 heartbeat_last_ms。
// timeout_ms: 超过该时间未更新则认为 watchdog 失活。
bool gpio_protect_heartbeat_is_wd_alive(int timeout_ms);

#endif
