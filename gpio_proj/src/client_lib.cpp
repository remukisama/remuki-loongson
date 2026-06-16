/**
 * @file client_lib.cpp
 * @brief 客户端接口实现
 *
 * 通过 POSIX 共享内存与 GPIO 看门狗线程通信。
 * 仅负责心跳发送与状态查询，不涉及硬件操作。
 */

#include "client_lib.hpp"
#include "shm_protocol.hpp"

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

namespace gpio {

static int        s_shm_fd = -1;
static ShmLayout *s_shm    = nullptr;
static uint32_t   s_heartbeat_counter = 0;

// ==================== 连接 / 断开 ====================

int init(void)
{
    if (s_shm) return 0;  // 已连接

    s_shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (s_shm_fd < 0) {
        printf("[GPIO Client] ERROR: cannot open shared memory (is watchdog running?)\n");
        return -1;
    }

    s_shm = (ShmLayout *)mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, s_shm_fd, 0);
    if (s_shm == MAP_FAILED) {
        printf("[GPIO Client] ERROR: mmap failed\n");
        s_shm = nullptr;
        return -1;
    }

    if (s_shm->version != SHM_VERSION) {
        printf("[GPIO Client] ERROR: version mismatch (shm=%d, expected=%d)\n",
               s_shm->version, SHM_VERSION);
        munmap(s_shm, SHM_SIZE);
        s_shm = nullptr;
        return -1;
    }

    s_heartbeat_counter = 0;
    printf("[GPIO Client] Connected to watchdog\n");
    return 0;
}

void deinit(void)
{
    if (s_shm && s_shm != MAP_FAILED) {
        // 停止心跳，触发看门狗进入安全模式
        s_shm->heartbeat_counter = 0;
        munmap(s_shm, SHM_SIZE);
        s_shm = nullptr;
    }
    if (s_shm_fd >= 0) {
        close(s_shm_fd);
        s_shm_fd = -1;
    }
    printf("[GPIO Client] Disconnected\n");
}

// ==================== 心跳 ====================

void heartbeat(void)
{
    if (!s_shm) return;
    s_heartbeat_counter++;
    s_shm->heartbeat_counter = s_heartbeat_counter;
}

bool is_safe_mode(void)
{
    if (!s_shm) return true;
    return s_shm->safe_mode != 0;
}

bool is_heartbeat_alive(void)
{
    if (!s_shm) return false;
    return s_shm->heartbeat_alive != 0;
}

}  // namespace gpio
