/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-03-03
@修改内容：新增线程接收模式
@注意事项：使用前需确保CAN接口已正确配置
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef __LQ_CNAFD_HPP
#define __LQ_CANFD_HPP

#include <iostream>
#include <string>
#include <functional>
#include <thread>
#include <atomic>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <signal.h>
#include <poll.h>

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

#define CANFD_MAX_DATA_LEN  64  // CANFD最大数据长度

// CAN接口名称宏定义
#define CAN0    "can0"
#define CAN1    "can1"

/****************************************************************************************************
 * @brief   枚举定义
 ****************************************************************************************************/

// 接收模式枚举
typedef enum
{
    CANFD_MODE_BLOCKING = 0,    // 阻塞模式
    CANFD_MODE_ASYNC    = 1,    // 异步信号模式（会中断主线程sleep）
    CANFD_MODE_THREAD   = 2,    // 独立线程模式（推荐，不影响主线程）
} ls_canfd_rx_mode_t;

/****************************************************************************************************
 * @brief   结构体定义
 ****************************************************************************************************/

// CAN数据帧结构体
typedef struct
{
    uint32_t can_id;                    // CAN ID
    uint8_t  len;                       // 数据长度
    uint8_t  data[CANFD_MAX_DATA_LEN];  // 数据
} ls_canfd_frame_t;

// 接收回调函数类型
typedef std::function<void(const ls_canfd_frame_t &frame)> ls_canfd_rx_callback_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class ls_canfd
{
public:
    // CAN FD 无参构造函数
    ls_canfd();
    // CAN FD 有参构造函数
    ls_canfd(const std::string &ifname, ls_canfd_rx_mode_t rx_mode = CANFD_MODE_THREAD, ls_canfd_rx_callback_t _cb = nullptr);
    // CAN FD 析构函数
    ~ls_canfd();

public:
    // 初始化CANFD
    bool canfd_init(const std::string &ifname, ls_canfd_rx_mode_t rx_mode = CANFD_MODE_THREAD, ls_canfd_rx_callback_t _cb = nullptr);

    int canfd_write_data (uint32_t _can_id, const uint8_t *_data, uint8_t _len);// 发送 CANFD 数据
    int canfd_write_frame(const ls_canfd_frame_t &_frame);                      // 发送 CANFD 数据帧

    int canfd_read_frame(ls_canfd_frame_t &_frame, int _timeout_ms = -1);       // 阻塞接收CANFD数据

    void set_rx_callback(ls_canfd_rx_callback_t _cb);       // 设置接收回调函数

    int get_socket() const { return this->m_socket; }       // 获取socket描述符

private:
    bool setup_async_mode();      // 设置异步信号模式
    void start_rx_thread();       // 启动接收线程
    void rx_thread_func();        // 接收线程函数

    static void signal_handler(int _signo);  // SIGIO信号处理函数

private:
    int                     m_socket;       // socket描述符
    std::string             m_ifname;       // 接口名称
    ls_canfd_rx_mode_t      m_rx_mode;      // 接收模式
    ls_canfd_rx_callback_t  m_rx_cb;        // 接收回调函数
    bool                    m_initialized;  // 初始化标志

    // 线程相关
    std::thread             m_rx_thread;    // 接收线程
    std::atomic<bool>       m_running;      // 线程运行标志

    static ls_canfd*        s_instance;     // 单例指针（用于信号处理）
};

#endif
