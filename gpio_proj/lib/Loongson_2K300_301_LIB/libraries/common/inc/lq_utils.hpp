#ifndef __LQ_UTILS_H__
#define __LQ_UTILS_H__ 

#include <stdint.h>
#include <type_traits>

// 通用工具函数

/* 通用模板: 判断数值是否在 [min_val, max_val] 闭区间范围内 */

template <typename T>
bool is_value_in_range(const T& value, const T& min_val, const T& max_val);

/* 通用模板：限幅函数 */

template <typename T>
T lq_limit(const T value, const T min_val, const T max_val);

/* 获取当前时间毫秒级 */
uint32_t lq_get_tick_ms(void);

#endif
