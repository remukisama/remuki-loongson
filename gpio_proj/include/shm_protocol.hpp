/**
 * @file shm_protocol.hpp
 * @brief 共享内存协议定义（看门狗架构）
 *
 * 定义小车线程与 GPIO 看门狗线程之间的共享内存布局。
 * 仅包含心跳与状态标志，不包含输出/输入通道数据。
 * 小车线程直接操作硬件，共享内存仅用于存活状态监测。
 *
 * 三方共用：小车线程（写心跳）、GPIO 看门狗（读心跳写状态）、屏幕线程（读状态）
 */

#ifndef GPIO_PROJ_SHM_PROTOCOL_HPP
#define GPIO_PROJ_SHM_PROTOCOL_HPP

#include <cstdint>

namespace gpio {

// ==================== 共享内存名称 ====================
static const char* SHM_NAME = "/gpio_proj_shm";

// ==================== 心跳参数 ====================
static const uint32_t HEARTBEAT_INTERVAL_MS = 1;    // 小车线程发送间隔（1ms）
static const uint32_t HEARTBEAT_TIMEOUT_MS  = 50;   // 超时阈值（50ms 未收到 → 异常）

// ==================== 共享内存布局 ====================

struct ShmLayout {
    // ---- 版本 ----
    uint32_t version;                       // 协议版本，用于兼容性检查

    // ---- 心跳（小车线程写入，看门狗+屏幕线程读取）----
    uint32_t heartbeat_counter;             // 小车线程每次调用 heartbeat() 递增

    // ---- 状态（GPIO 看门狗写入，屏幕线程读取）----
    uint8_t  safe_mode;                     // 1=安全模式（心跳超时，看门狗已接管）
    uint8_t  heartbeat_alive;               // 1=活跃, 0=超时
    uint8_t  reserved[2];                   // 对齐填充

    // ---- 时间戳（GPIO 看门狗写入）----
    uint32_t heartbeat_last_ms;             // 最后心跳时间（毫秒）

    // ---- 显存区域（预留，屏幕工程开发时启用）----
    // uint32_t framebuffer_size;           // 显存区域大小
    // uint8_t  framebuffer[];              // 柔性数组，屏幕分辨率决定大小
};

static const uint32_t SHM_VERSION = 2;      // 版本号更新为 2（看门狗架构）
static const uint32_t SHM_SIZE    = sizeof(ShmLayout);

}  // namespace gpio

#endif /* GPIO_PROJ_SHM_PROTOCOL_HPP */
