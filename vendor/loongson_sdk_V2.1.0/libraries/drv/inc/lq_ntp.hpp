#ifndef __LQ_NTP_HPP
#define __LQ_NTP_HPP

#include <iostream>
#include <string>
#include <fcntl.h>
#include <stdlib.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/time.h>
#include <mutex>

class lq_ntp
{
public:
    lq_ntp();       // 构造函数
    ~lq_ntp();      // 析构函数

    lq_ntp(const lq_ntp &ntp) = delete;             // 复制构造函数
    lq_ntp &operator=(const lq_ntp &ntp) = delete;  // 赋值运算符
    lq_ntp(lq_ntp &&ntp) = delete;                  // 移动构造函数
    lq_ntp &operator=(lq_ntp &&ntp) = delete;       // 移动赋值运算符

public:
    uint64_t get_local_time_s (void);   // 获取以秒为单位时间戳
    uint64_t get_local_time_ms(void);   // 获取以毫秒为单位时间戳
    uint64_t get_local_time_us(void);   // 获取以微秒为单位时间戳

    std::string get_local_time_str  (void);             // 获取本地时间，格式：YYYY-MM-DD HH:MM:SS
    std::string get_local_year      (uint32_t *_year);  // 获取本地时间戳的年份
    std::string get_local_month     (uint8_t  *_month); // 获取本地时间戳的月份
    std::string get_local_day       (uint16_t *_day);   // 获取本地时间戳的日期
    std::string get_local_hour      (uint8_t  *_hour);  // 获取本地时间戳的小时
    std::string get_local_minute    (uint8_t  *_minute);// 获取本地时间戳的分钟
    std::string get_local_second    (uint8_t  *_second);// 获取本地时间戳的秒数

    // 主动同步时间，重新从 NTP 服务器获取时间并设置系统时间
    bool sync_ntp_time(void);

private:
    int  get_ntp_time   (uint32_t *_unix_time); // 从 NTP 服务器获取 Unix 时间戳
    int  set_system_time(uint32_t _unix_time);  // 设置系统时间
    void sync_hw_clock  (void);                 // 同步到硬件时间

    bool get_system_tm  (struct tm& out_tm);    // 获取系统时间的 tm 结构体

private:
    std::mutex        mtx_;                                     // 互斥锁
    const std::string ntp_server_           = "182.92.12.11";   // NTP 服务器地址
    const int         ntp_port_             = 123;              // NTP 端口号
    const uint64_t    ntp_timestamp_delta_  = 2208988800ULL;    // NTP 时间戳偏移量，单位：秒
    const uint32_t    ntp_time_zone_offset_ = (8 * 3600);       // NTP 时区偏移量，单位：秒
};

#endif
