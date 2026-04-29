#include <stdint.h>
#ifndef __LQ_REG_QUADRATURE_ENCODER_HPP
#define __LQ_REG_QUADRATURE_ENCODER_HPP

#include "lq_reg_gpio.hpp"

class ls_quadrature_encoder {
public:
    // 构造函数，传入对应的内核字符设备索引 (例如: 0 表示 /dev/lq_encoder0, 1 表示 /dev/lq_encoder1)
    ls_quadrature_encoder(uint8_t dev_id);
    ~ls_quadrature_encoder();

    float encoder_get_count(void);              // 获取与上次调用相比的脉冲增量(Δcount)
    int8_t encoder_get_direction(void);         // 获取实时方向 (1: 正转, -1: 反转, 0: 未知/停止)
    void  encoder_reset_counter(void);          // 重置编码器计数器

    ls_quadrature_encoder(const ls_quadrature_encoder&) = delete;
    ls_quadrature_encoder& operator=(const ls_quadrature_encoder&) = delete;

private:
    int fd;                                     // 字符设备文件描述符
    long long last_count;
    uint64_t last_time_us;                      // 作为首次调用基线标志使用(0:未初始化)
};

#endif
