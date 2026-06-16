/**
 * @file test_client.cpp
 * @brief 客户端接口测试 — 验证心跳与安全模式状态
 *
 * 使用方法：
 *   ./gpio_manager &       # 先启动看门狗
 *   ./test_client          # 再运行测试
 *
 * 测试内容：
 *   1. 连接共享内存
 *   2. 持续发送心跳，观察状态
 *   3. 模拟心跳停止，验证看门狗进入安全模式
 *   4. 恢复心跳，验证退出安全模式
 */

#include "../src/client_lib.hpp"
#include "../include/shm_protocol.hpp"
#include <cstdio>
#include <unistd.h>

int main()
{
    printf("========================================\n");
    printf("  GPIO Client 接口测试\n");
    printf("  gpio_manager 必须已在后台运行\n");
    printf("========================================\n");

    // 连接
    if (gpio::init() != 0) {
        printf("[FAIL] 无法连接看门狗\n");
        return -1;
    }
    printf("[OK] 已连接看门狗\n\n");

    // === 测试 1: 正常心跳 ===
    printf("--- 测试 1: 正常心跳 (5秒) ---\n");
    for (int i = 0; i < 500; i++) {
        gpio::heartbeat();
        usleep(10000);  // 10ms
        if (i % 100 == 99) {
            printf("  [%d ms] safe_mode=%d, alive=%d\n",
                   (i + 1) * 10,
                   gpio::is_safe_mode() ? 1 : 0,
                   gpio::is_heartbeat_alive() ? 1 : 0);
        }
    }
    printf("  结果: safe_mode=%d (应为0), alive=%d (应为1)\n\n",
           gpio::is_safe_mode() ? 1 : 0,
           gpio::is_heartbeat_alive() ? 1 : 0);

    // === 测试 2: 停止心跳，模拟小车线程卡死 ===
    printf("--- 测试 2: 停止心跳 200ms (模拟卡死) ---\n");
    printf("  停止心跳中...\n");
    usleep(200000);  // 200ms 不发心跳
    printf("  结果: safe_mode=%d (应为1), alive=%d (应为0)\n",
           gpio::is_safe_mode() ? 1 : 0,
           gpio::is_heartbeat_alive() ? 1 : 0);

    if (gpio::is_safe_mode()) {
        printf("  [OK] 看门狗已进入安全模式，硬件已写入安全值\n\n");
    } else {
        printf("  [FAIL] 看门狗未进入安全模式\n\n");
    }

    // === 测试 3: 恢复心跳 ===
    printf("--- 测试 3: 恢复心跳 ---\n");
    for (int i = 0; i < 50; i++) {
        gpio::heartbeat();
        usleep(10000);  // 10ms
    }
    printf("  结果: safe_mode=%d (应为0), alive=%d (应为1)\n",
           gpio::is_safe_mode() ? 1 : 0,
           gpio::is_heartbeat_alive() ? 1 : 0);

    if (!gpio::is_safe_mode()) {
        printf("  [OK] 看门狗已退出安全模式\n\n");
    } else {
        printf("  [FAIL] 看门狗仍在安全模式\n\n");
    }

    // === 测试 4: 再次停止心跳，验证可重复触发 ===
    printf("--- 测试 4: 再次停止心跳 200ms ---\n");
    usleep(200000);
    printf("  结果: safe_mode=%d (应为1)\n",
           gpio::is_safe_mode() ? 1 : 0);

    // 恢复
    for (int i = 0; i < 50; i++) {
        gpio::heartbeat();
        usleep(10000);
    }

    // 清理
    printf("\n--- 测试结束 ---\n");
    gpio::deinit();

    printf("========================================\n");
    printf("  全部测试完成\n");
    printf("========================================\n");
    return 0;
}
