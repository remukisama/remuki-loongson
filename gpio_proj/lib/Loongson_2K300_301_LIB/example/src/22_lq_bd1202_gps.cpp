#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_bd1202_gps_demo.cpp
 * @brief   BD1202 GPS模块 demo.
 * @author  龙邱科技-012
 * @date    2026-04-21
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 BD1202 GPS模块功能.
 ********************************************************************************/

/********************************************************************************
 * @brief   BD1202 GPS模块 demo.
 * @param   none.
 * @return  none.
 * @note    本 demo 实现 BD1202 GPS模块功能.
 ********************************************************************************/
void lq_bd1202_gps_demo(void)
{
    char txt[32];
    int Lon_IZ = 0, Lon_IX = 0, Lat_IZ = 0, Lat_IX = 0;
    double Lon = 0.0, Lat = 0.0;

    // 初始化串口，使用线程接收模式
    lq_bd_gps gps(UART1_PIN42);

    while (ls_system_running.load()) // 主循环
    {
        // 以逗号分割GPS_Buffer内RMC语句，拆分字段存入Save_Data成员
        gps.parse_gps_buffer();
        // 获取并打印时间
        sprintf(txt, "T:%s", gps.get_utc_time());
        printf("%s\n", txt);

        // 获取原始经纬度字符串拆分整数、小数部分
        if (gps.get_int_data(&Lon_IZ, &Lon_IX, &Lat_IZ, &Lat_IX) == 0)
        {
            Lon_IZ = 0;
            Lon_IX = 0;
            Lat_IZ = 0;
            Lat_IX = 0;
        }
        sprintf(txt, "N:%d.%d", Lat_IZ, Lat_IX); // 转化成数据
        printf("%s\n", txt);
        sprintf(txt, "E:%d.%d", Lon_IZ, Lon_IX); // 数据
        printf("%s\n", txt);

        // 获取度分格式转标准十进制度浮点坐标
        if (gps.get_double_data(&Lon, &Lat) == 0)
        {
            Lon = 0.0;
            Lat = 0.0;
        }
        sprintf(txt, "N:%f", Lat); // 转化成数据
        printf("%s\n", txt);
        sprintf(txt, "E:%f", Lon); // 数据
        printf("%s\n\n", txt);
        usleep(100 * 1000);
    }
}
