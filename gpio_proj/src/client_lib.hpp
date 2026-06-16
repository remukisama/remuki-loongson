/**
 * @file client_lib.hpp
 * @brief 客户端接口 — 小车线程通过此接口与 GPIO 看门狗通信
 *
 * 精简版：仅提供心跳与状态查询，不承担硬件输出转发。
 * 小车线程直接操作硬件（ATIM PWM、GPIO、编码器），本接口仅用于：
 *   1. 连接共享内存
 *   2. 发送心跳（每 1ms 调用一次）
 *   3. 查询安全模式状态
 */

#ifndef GPIO_PROJ_CLIENT_LIB_HPP
#define GPIO_PROJ_CLIENT_LIB_HPP

#include <cstdint>

namespace gpio {

/// 连接共享内存
int  init(void);

/// 断开连接
void deinit(void);

/// 发送心跳（每 1ms 调用一次，递增心跳计数器）
void heartbeat(void);

/// 查询是否处于安全模式（心跳丢失，看门狗已接管）
bool is_safe_mode(void);

/// 查询看门狗是否认为小车线程存活
bool is_heartbeat_alive(void);

}  // namespace gpio

#endif /* GPIO_PROJ_CLIENT_LIB_HPP */
