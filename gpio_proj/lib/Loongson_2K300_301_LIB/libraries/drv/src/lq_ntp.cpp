#include "lq_ntp.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @fn      lq_ntp::lq_ntp();
 * @brief   构造函数，用于初始化 lq_ntp 对象.
 * @param   none.
 * @return  none.
 * @date    2026-03-13.
 ********************************************************************************/
lq_ntp::lq_ntp()
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    // 获取网络时间
    uint32_t unix_time = 0;
    if (this->get_ntp_time(&(unix_time)) != 0) {
        lq_log_warn("获取网络时间失败，使用系统默认时间");
        return;
    }
    // 自动 +8 小时 → 北京时间
    unix_time += this->ntp_time_zone_offset_;
    // 设置系统时间
    if (this->set_system_time(unix_time) != 0) {
        lq_log_error("设置系统时间失败");
        return;
    }
    // 同步到硬件时间
    this->sync_hw_clock();
    lq_log_info("同步 NTP 时间成功");
}

/********************************************************************************
 * @fn      lq_ntp::~lq_ntp();
 * @brief   析构函数，用于释放 lq_ntp 对象所占用的资源.
 * @param   none.
 * @return  none.
 * @date    2026-03-13.
 ********************************************************************************/
lq_ntp::~lq_ntp()
{
}

/********************************************************************************
 * @fn      bool lq_ntp::sync_ntp_time(void);
 * @brief   主动同步时间，重新从 NTP 服务器获取时间并设置系统时间.
 * @param   none.
 * @return  true: 成功, false: 失败.
 * @date    2026-03-13.
 ********************************************************************************/
bool lq_ntp::sync_ntp_time(void)
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    uint32_t unix_time = 0;
    if (this->get_ntp_time(&unix_time) != 0) {
        lq_log_error("NTP 时间同步失败");
        return false;
    }
    uint32_t local_time = unix_time + this->ntp_time_zone_offset_;
    this->set_system_time(local_time);
    this->sync_hw_clock();
    lq_log_info("NTP 时间同步成功");
    return true;
}

/********************************************************************************
 * @fn      uint64_t lq_ntp::get_local_time_s(void);
 * @brief   获取以秒为单位时间戳.
 * @param   none.
 * @return  以为秒为单位的时间戳.
 * @date    2026-03-13.
 ********************************************************************************/
uint64_t lq_ntp::get_local_time_s(void)
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    struct timeval tv{};
    gettimeofday(&tv, nullptr);
    return (uint64_t)tv.tv_sec;
}

/********************************************************************************
 * @fn      uint64_t lq_ntp::get_local_time_ms(void);
 * @brief   获取以毫秒为单位时间戳.
 * @param   none.
 * @return  以毫秒为单位的时间戳.
 * @date    2026-03-13.
 ********************************************************************************/
uint64_t lq_ntp::get_local_time_ms(void)
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    struct timeval tv{};
    gettimeofday(&tv, nullptr);
    return (uint64_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

/********************************************************************************
 * @fn      uint64_t lq_ntp::get_local_time_us(void);
 * @brief   获取以微秒为单位时间戳.
 * @param   none.
 * @return  以微秒为单位的时间戳.
 * @date    2026-03-13.
 ********************************************************************************/
uint64_t lq_ntp::get_local_time_us(void)
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    struct timeval tv{};
    gettimeofday(&tv, nullptr);
    return (uint64_t)(tv.tv_sec * 1000000 + tv.tv_usec);
}

/********************************************************************************
 * @fn      std::string lq_ntp::get_local_time_str(void);
 * @brief   获取本地时间，格式：YYYY-MM-DD HH:MM:SS.
 * @param   none.
 * @return  本地时间字符串.
 * @date    2026-03-13.
 ********************************************************************************/
std::string lq_ntp::get_local_time_str(void)
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    time_t now = time(nullptr);
    struct tm local_time{};
    localtime_r(&now, &local_time);
    char buffer[20];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &local_time);
    return std::string(buffer);
}

/********************************************************************************
 * @fn      std::string lq_ntp::get_local_year(uint32_t *_year);
 * @brief   获取本地时间戳的年份.
 * @param   _year: 输出参数，用于存储年份.
 * @return  std::string: 返回获取到的年份字符串.
 * @date    2026-03-13.
 ********************************************************************************/
std::string lq_ntp::get_local_year(uint32_t *_year)
{
    struct tm ntp_time{};
    this->get_system_tm(ntp_time);
    if (_year != nullptr) {
        *_year = ntp_time.tm_year + 1900;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", ntp_time.tm_year + 1900);
    return std::string(buf);
}

/********************************************************************************
 * @fn      std::string lq_ntp::get_local_month(uint8_t *_month);
 * @brief   获取本地时间戳的月份.
 * @param   _month: 输出参数，用于存储月份.
 * @return  std::string: 返回月份的字符串表示.
 * @date    2026-03-13.
 ********************************************************************************/
std::string lq_ntp::get_local_month(uint8_t *_month)
{
    struct tm ntp_time{};
    this->get_system_tm(ntp_time);
    if (_month != nullptr) {
        *_month = ntp_time.tm_mon + 1;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%02u", ntp_time.tm_mon + 1);
    return std::string(buf);
}

/********************************************************************************
 * @fn      std::string lq_ntp::get_local_day(uint16_t *_day);
 * @brief   获取本地时间戳的日期.
 * @param   _day: 输出参数，用于存储日期.
 * @return  std::string: 返回日期字符串.
 * @date    2026-03-13.
 ********************************************************************************/
std::string lq_ntp::get_local_day(uint16_t *_day)
{
    struct tm ntp_time{};
    this->get_system_tm(ntp_time);
    if (_day != nullptr) {
        *_day = ntp_time.tm_mday;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%02u", ntp_time.tm_mday);
    return std::string(buf);
}

/********************************************************************************
 * @fn      std::string lq_ntp::get_local_hour(uint8_t *_hour);
 * @brief   获取本地时间戳的小时.
 * @param   _hour: 输出参数，用于存储小时.
 * @return  std::string: 返回格式化后的小时字符串.
 * @date    2026-03-13.
 ********************************************************************************/
std::string lq_ntp::get_local_hour(uint8_t *_hour)
{
    struct tm ntp_time{};
    this->get_system_tm(ntp_time);
    if (_hour != nullptr) {
        *_hour = ntp_time.tm_hour;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%02u", ntp_time.tm_hour);
    return std::string(buf);
}

/********************************************************************************
 * @fn      std::string lq_ntp::get_local_minute(uint8_t *_minute);
 * @brief   获取本地时间戳的分钟.
 * @param   _minute: 输出参数，用于存储分钟.
 * @return  std::string: 返回格式化后的分钟字符串.
 * @date    2026-03-13.
 ********************************************************************************/
std::string lq_ntp::get_local_minute(uint8_t *_minute)
{
    struct tm ntp_time{};
    this->get_system_tm(ntp_time);
    if (_minute != nullptr) {
        *_minute = ntp_time.tm_min;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%02u", ntp_time.tm_min);
    return std::string(buf);
}

/********************************************************************************
 * @fn      std::string lq_ntp::get_local_second(uint8_t *_second);
 * @brief   获取本地时间戳的秒数.
 * @param   _second: 输出参数，用于存储秒数.
 * @return  std::string: 返回格式化后的字符串.
 * @date    2026-03-13.
 ********************************************************************************/
std::string lq_ntp::get_local_second(uint8_t *_second)
{
    struct tm ntp_time{};
    this->get_system_tm(ntp_time);
    if (_second != nullptr) {
        *_second = ntp_time.tm_sec;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%02u", ntp_time.tm_sec);
    return std::string(buf);
}

/********************************************************************************
 * @fn      int lq_ntp::get_ntp_time(uint32_t *_unix_time);
 * @brief   从 NTP 服务器获取 Unix 时间戳.
 * @param   _unix_time: 输出参数，用于存储获取到的 Unix 时间戳.
 * @return  0: 获取成功; -1: 获取失败.
 * @date    2026-03-13.
 ********************************************************************************/
int lq_ntp::get_ntp_time(uint32_t *_unix_time)
{
    if (_unix_time == nullptr) {
        lq_log_error("Invalid pointer");
    }
    uint8_t ntp_packet[48] = {0};   // NTP 数据包固定 48 字节
    // 创建 UDP 套接字
    int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0) {
        lq_log_error("socket failed");
        return -1;
    }
    // 超时设置
    struct timeval tv{};
    tv.tv_sec  = 1;
    tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    // 直接填入 IP 地址
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(ntp_port_);
    if (inet_pton(AF_INET, ntp_server_.c_str(), &server_addr.sin_addr) < 1) {
        lq_log_error("inet_pton failed");
        close(sockfd);
        return -1;
    }
    // NTP 协议要求：第一个字节 = 0x1B
    ntp_packet[0] = 0x1B;
    // 发送请求
    if (sendto(sockfd, ntp_packet, sizeof(ntp_packet), 0, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        lq_log_error("sendto failed");
        close(sockfd);
        return -1;
    }
    // 接收响应
    if (recv(sockfd, ntp_packet, sizeof(ntp_packet), 0) < 0) {
        lq_log_error("recv failed");
        close(sockfd);
        return -1;
    }
    // 提取 NTP 时间戳（网络字节序转主机序）
    uint32_t ntp_time = (ntohl(*((uint32_t*)(ntp_packet + 40))));
    // 转换未 Unix 时间戳
    *_unix_time = ntp_time - ntp_timestamp_delta_;
    close(sockfd);
    return 0;
}

/********************************************************************************
 * @fn      int lq_ntp::set_system_time(uint32_t _unix_time);
 * @brief   设置系统时间.
 * @param   _unix_time: 要设置的 Unix 时间戳.
 * @return  0: 设置成功; -1: 设置失败.
 * @date    2026-03-13.
 ********************************************************************************/
int lq_ntp::set_system_time(uint32_t _unix_time)
{
    struct timeval tv{};
    tv.tv_sec  = _unix_time;
    tv.tv_usec = 0;
    // 设置系统时间
    if (settimeofday(&tv, nullptr) < 0) {
        lq_log_error("settimeofday failed");
        return -1;
    }
    return 0;
}

/********************************************************************************
 * @fn      void lq_ntp::sync_hw_clock(void);
 * @brief   同步到硬件时间.
 * @param   none.
 * @return  none.
 * @date    2026-03-13.
 ********************************************************************************/
void lq_ntp::sync_hw_clock(void)
{
    // 同步到硬件时间
    system("hwclock -w");
}

/********************************************************************************
 * @fn      bool lq_ntp::get_system_tm(struct tm& out_tm);
 * @brief   获取系统时间的 tm 结构体.
 * @param   out_tm: 输出参数，用于存储获取到的 tm 结构体.
 * @return  true: 获取成功，false: 获取失败.
 * @date    2026-03-13.
 ********************************************************************************/
bool lq_ntp::get_system_tm(struct tm& out_tm)
{
    time_t now = time(nullptr);
    if (localtime_r(&now, &out_tm) == nullptr) {
        lq_log_error("获取系统时间失败");
        return false;
    }
    return true;
}
