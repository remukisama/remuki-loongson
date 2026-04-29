#ifndef __LQ_UTILS_H__
#define __LQ_UTILS_H__ 

#include <stdint.h>
#include <type_traits>

// 通用工具函数

/* 通用模板: 判断数值是否在 [min_val, max_val] 闭区间范围内 */

template <typename T>
bool is_value_in_range(const T& value, const T& min_val, const T& max_val);

#endif
