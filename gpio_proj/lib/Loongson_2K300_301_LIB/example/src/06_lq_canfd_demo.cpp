#include "lq_all_demo.hpp"
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>

/********************************************************************************
 * @file    lq_canfd_demo.cpp
 * @brief   CANFD 测试.
 * @author  龙邱科技-006
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 CANFD 功能，用于测试 CANFD 控制器的基本功能.
 ********************************************************************************/

// 获取当前时间字符串（格式： YYYY-MM-DD HH:mm:ss）
static std::string get_time_str(void)
{
    // 获取时间对象
    static lq_ntp g_ntp;
    return g_ntp.get_local_time_str();
}

// CAN接收回调函数（在独立线程中运行）
void CAN_RxCallback(const ls_canfd_frame_t &frame)
{
    printf("[%s] 收到: CAN ID 0x%03X, 长度 %d, 数据: ", 
           get_time_str().c_str(), frame.can_id, frame.len);
    for (int i = 0; i < frame.len; i++) {
        printf("%02X ", frame.data[i]);
    }
    printf("\n");
}

/********************************************************************************
 * @brief   CANFD 测试程序.
 * @param   none.
 * @return  none.
 * @note    使用独立线程模式，测试CANFD的收发帧功能.
 ********************************************************************************/
void lq_canfd_demo(void)
{
    // CANFD对象
    ls_canfd g_can;

    // 初始化CAN，使用独立线程模式，接收回调函数为CAN_RxCallback
    if (!g_can.canfd_init(CAN1, CANFD_MODE_THREAD, CAN_RxCallback)) {
        printf("CAN初始化失败!\n");
        return;
    }

    printf("主程序开始运行...\n");
    printf("CAN数据在独立线程中接收处理\n\n");

    // 主循环
    int count = 0;
    while (ls_system_running.load()) {
        printf("[%s] 主程序运行中... count = %d\n", get_time_str().c_str(), count++);
        // 发送数据
        uint8_t tx_data[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x11, 0x22, 0x33, 0x44};
        g_can.canfd_write_data(0x123, tx_data, sizeof(tx_data));
        printf("[%s] 发送数据: CAN ID 0x123, 长度 %zu\n", get_time_str().c_str(), sizeof(tx_data));
        sleep(2);
    }
    return;
}
