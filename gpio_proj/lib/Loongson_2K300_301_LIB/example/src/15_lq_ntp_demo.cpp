#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_ntp_demo.cpp
 * @brief   NTP 测试.
 * @author  龙邱科技-012
 * @date    2026-03-18
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 NTP 功能，用于测试 NTP 控制器的基本功能.
 ********************************************************************************/

/********************************************************************************
 *  @brief   ntp 获取网络时间并更新到系统中.
 *  @param   none.
 *  @return  none.
 *  @note    创建 lq_ntp 变量，构造函数中会自动 NTP 获取网络时间并更新到系统中.
 *! @note    需要连接网络，否则会获取失败，后续获取时间会获取本地时间.
 ********************************************************************************/
void lq_ntp_demo(void)
{
    // 初始化 NTP 时间同步
    lq_ntp ntp;

    while (ls_system_running.load())
    {
        std::cout << "获取秒数时间戳\t: " << ntp.get_local_time_s() << std::endl;
        std::cout << "获取毫秒时间戳\t: " << ntp.get_local_time_ms() << std::endl;
        std::cout << "获取微秒时间戳\t: " << ntp.get_local_time_us() << std::endl;
        std::cout << "本地时间字符串\t: " << ntp.get_local_time_str() << std::endl;
        std::cout << "本地时间年\t: " << ntp.get_local_year(nullptr) << std::endl;
        std::cout << "本地时间月\t: " << ntp.get_local_month(nullptr) << std::endl;
        std::cout << "本地时间日\t: " << ntp.get_local_day(nullptr) << std::endl;
        std::cout << "本地时间时\t: " << ntp.get_local_hour(nullptr) << std::endl;
        std::cout << "本地时间分\t: " << ntp.get_local_minute(nullptr) << std::endl;
        std::cout << "本地时间秒\t: " << ntp.get_local_second(nullptr) << std::endl << std::endl;
        usleep(100*100);
    }
}
