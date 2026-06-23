#ifndef GPIO_PROTECT_HEARTBEAT_HPP_
#define GPIO_PROTECT_HEARTBEAT_HPP_

void gpio_protect_heartbeat_init(void);
void gpio_protect_heartbeat_pulse(void);
void gpio_protect_heartbeat_deinit(void);
bool gpio_protect_heartbeat_require_ready(int timeout_ms);

#endif
