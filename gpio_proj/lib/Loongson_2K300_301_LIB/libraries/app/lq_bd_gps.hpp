#ifndef __LQ_BD_GPS_HPP__
#define __LQ_BD_GPS_HPP__

#include "lq_drv_inc.hpp"

/********************************************************************************
 * @brief   纬度南北标识枚举
 * @param   N : 北纬
 * @param   S : 南纬
 * @return  none
 * @note    用于标记Position_t结构体纬度方向
 ********************************************************************************/
typedef enum
{
    N = 0,
    S
} lat_t;

/********************************************************************************
 * @brief   经度东西标识枚举
 * @param   E : 东经
 * @param   W : 西经
 * @return  none
 * @note    用于标记Position_t结构体经度方向
 ********************************************************************************/
typedef enum
{
    E = 0,
    W
} lon_t;

/********************************************************************************
 * @brief   坐标解析结构体，存储拆分后的经纬度整数、小数、方向、浮点坐标
 * @param   Lon_dir     : 经度方向字符(E/W)
 * @param   Lat_dir     : 纬度方向字符(N/S)
 * @param   Lon_front   : 经度整数部分（BD原始格式：dddmm.xxxxxx，dddmm存在此处）
 * @param   Lon_back    : 经度小数点后小数部分
 * @param   Lat_front   : 纬度整数部分（ddmm.xxxxxx，ddmm存在此处）
 * @param   Lat_back    : 纬度小数点后小数部分
 * @param   lon         : 换算后十进制真实经度(°)
 * @param   lat         : 换算后十进制真实纬度(°)
 * @return  none
 * @note    BD/RMC语句专用坐标存储，度分格式转十进制存放
 ********************************************************************************/
typedef struct
{
    char        Lon_dir;
    char        Lat_dir;
    int32_t     Lon_front;
    int32_t     Lon_back;
    int32_t     Lat_front;
    int32_t     Lat_back;
    double      lon;
    double      lat;
} position_t;

/********************************************************************************
 * @brief   GPS/北斗原始帧解析缓存结构体，存储RMC语句拆分字段
 * @param   GPS_Buffer[128] : 串口接收完整一帧NMEA语句缓存
 * @param   isGetData       : 帧接收完成标志 1=收完一帧 0=未收完
 * @param   isParseData     : 帧解析完成标志 1=解析完毕 0=未解析
 * @param   UTCTime[11]     : UTC时间，解析后转为北京时间
 * @param   latitude[11]    : NMEA原始纬度字符串(ddmm.xxxxxx)
 * @param   N_S[2]          : 纬度方向 N北纬/S南纬
 * @param   longitude[12]   : NMEA原始经度字符串(dddmm.xxxxxx)
 * @param   E_W[2]          : 经度方向 E东经/W西经
 * @param   isUsefull       : 定位有效性 A有效/V无效
 * @return  none
 * @note    适配$GNRMC/$BDRMC NMEA0183协议
 ********************************************************************************/
typedef struct SaveData
{
    char    GPS_Buffer[128]; // 完整数据
    char    isGetData;       // 是否获取到GPS数据
    char    isParseData;     // 是否解析完成
    char    UTCTime[11];     // UTC时间
    char    latitude[11];    // 纬度
    char    N_S[2];          // N/S
    char    longitude[12];   // 经度
    char    E_W[2];          // E/W
    char    isUsefull;       // 定位信息是否有效
} save_data_t;

void Test_BD(void);

/********************************************************************************
 * @brief BD/GPS模块解析类，基于串口接收NMEA-0183协议，完成RMC报文解析
 * @note 内置串口实例、接收缓存与数据存储结构体，提供时间、坐标获取接口
 ********************************************************************************/
class lq_bd_gps
{
public:
    lq_bd_gps(uart_pin_t _pin); // GPS 构造函数，初始化对应引脚串口并绑定接收回调
    ~lq_bd_gps();               // GPS 析构函数

public:
    // 拆分 RMC 报文，提取 UTC、经纬度等字段存入数据缓存
    void parse_gps_buffer(void);

    // 原始经纬度字符串拆分整数、小数部分
    char get_int_data(int *Lon_Z, int *Lon_X, int *Lat_Z, int *Lat_X);

    // 度分格式转标准十进制度浮点坐标
    char get_double_data(double *Lon, double *Lat);

    // 获取北京时间字符串指针
    const char *get_utc_time(void);

private:
    int BDRMC_getdata(void);                // 检索 $BDRMC 帧，解析坐标存入 point_p 结构体

    void read_data(const uint8_t data);     // 串口单字节接收回调，拼接一帧 NMEA 报文

private:
    ls_uart     uart;               // UART 对象实例

    uint8_t     read_buffer[128];   // UART 接收缓冲区
    position_t  point_p;            // 坐标解析结构体实例
    save_data_t Save_Data;          // GPS 原始帧解析缓存结构体实例

};

#endif
