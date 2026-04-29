#include "lq_utils.hpp"

/********************************************************************************
 * @brief   通用模板: 判断数值是否在 [min_val, max_val] 闭区间范围内
 * @param   value   : 待判断的数值
 * @param   min_val : 范围最小值(需与 value 同类型)
 * @param   max_val : 范围最大值(需与 value 同类型)
 * @return  true - 在范围内; false - 不在范围内
 * @note    1. 需保证 min_val <= max_val， 否则直接返回false
 * @note    2. 模板仅接受算术类型(数值类型), 非数值类型编译时会报错
 * @note    3. 常用模板实例已经添加, 若有其他需要可按照模板自行添加
 ********************************************************************************/
template <typename T>
bool is_value_in_range(const T& value, const T& min_val, const T& max_val)
{
    static_assert(std::is_arithmetic<T>::value, "is_value_in_range: Only arithmetic types are allowed!");
    // 若最小值大于最大值, 直接返回false
    if (min_val > max_val) {
        return false;
    }
    return (value >= min_val && value <= max_val);
}
template bool is_value_in_range<uint8_t >(const uint8_t &, const uint8_t &, const uint8_t &);
template bool is_value_in_range<uint16_t>(const uint16_t&, const uint16_t&, const uint16_t&);
template bool is_value_in_range<uint32_t>(const uint32_t&, const uint32_t&, const uint32_t&);
template bool is_value_in_range<uint64_t>(const uint64_t&, const uint64_t&, const uint64_t&);

template bool is_value_in_range<int8_t >(const int8_t &, const int8_t &, const int8_t &);
template bool is_value_in_range<int16_t>(const int16_t&, const int16_t&, const int16_t&);
template bool is_value_in_range<int32_t>(const int32_t&, const int32_t&, const int32_t&);
template bool is_value_in_range<int64_t>(const int64_t&, const int64_t&, const int64_t&);

