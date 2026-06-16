#include "lq_bd_gps.hpp"

/********************************************************************************
 * @brief    GPS对象构造，初始化串口并绑定接收回调
 * @param    _pin：硬件串口引脚定义
 * @return   无
 * @note     串口参数：115200 8N1、线程接收模式；延时等待GPS上电稳定
 ********************************************************************************/
lq_bd_gps::lq_bd_gps(uart_pin_t _pin)
    : uart(_pin, 115200, LS_UART_DATA8, LS_UART_STOP1, LS_UART_PARITY_NONE, UART_MODE_THREAD,
        [this](uint8_t ch){ this->read_data(ch); })
{
    usleep(100 * 1000); // 等待GPS模块上电稳定
}

/********************************************************************************
 * @brief    GPS析构函数
 * @param    无
 * @return   无
 * @note     内部uart成员自动析构释放串口资源，无需手动释放
 ********************************************************************************/
lq_bd_gps::~lq_bd_gps()
{
}

/********************************************************************************
 * @brief    检索缓冲区$BDRMC帧，解析经纬度并换算十进制坐标
 * @param    无
 * @return   int：1解析成功，0未检索到BDRMC报文
 * @note     NMEA格式 ddmm.xxxx，拆分整数小数后换算为十进制度存入point_p
 ********************************************************************************/
int lq_bd_gps::BDRMC_getdata(void)
{
    const char *p;
    int a, b;
    char c;

    double X, Y;

    if ((p = strstr((const char *)(this->read_buffer), "$BDRMC")) != NULL)
        sscanf(p, "$BDRMC,%d.%d,%c,%d.%d,%c,%d.%d,%c,", &a, &b, &c,
            &(this->point_p.Lat_front), &(this->point_p.Lat_back),
            &(this->point_p.Lat_dir)  , &(this->point_p.Lon_front),
            &(this->point_p.Lon_back) , &(this->point_p.Lon_dir));
    else
        return 0; // 数据错误

    X = (double)(this->point_p.Lat_front) / 100;
    this->point_p.lat = X + (double)(this->point_p.Lat_back) / 10000000;
    Y = (double)(this->point_p.Lon_front) / 100;
    this->point_p.lon = Y + (double)(this->point_p.Lon_back) / 10000000;

    return 1;
}

/********************************************************************************
 * @brief   以逗号分割GPS_Buffer内RMC语句，拆分字段存入Save_Data成员
 * @param   none
 * @return  none
 * @note    i=1 UTC时间+8时区；i3~i6经纬度与方向；死循环阻塞代表字段缺失解析异常
 ********************************************************************************/
void lq_bd_gps::parse_gps_buffer(void)
{
    char *subString;
    char *subStringNext;
    char i = 0;
    if (this->Save_Data.isGetData) // 如果字符串不为空
    {
        for (i = 0; i <= 6; i++) // 循环7次
        {
            if (i == 0)
            {
                if ((subString = strchr(this->Save_Data.GPS_Buffer, ',')) == NULL) // 没有检测到逗号
                    while (ls_system_running.load())
                        ; // 解析错误
            }
            else // 检测到逗号，返回逗号的位置
            {
                subString++;                                          // 位置加1（定位到逗号的后一位）
                if ((subStringNext = strchr(subString, ',')) != NULL) // 定位下一个逗号的位置
                {
                    char usefullBuffer[2];
                    switch (i)
                    {
                    case 1:
                        memcpy(this->Save_Data.UTCTime, subString, subStringNext - subString); // 两个逗号之间为 时间信息 并转换成北京时间
                        this->Save_Data.UTCTime[1] = this->Save_Data.UTCTime[1] + 8;
                        if (this->Save_Data.UTCTime[1] > '9')
                        {
                            this->Save_Data.UTCTime[0]++;
                            if (this->Save_Data.UTCTime[0] == '3')
                                this->Save_Data.UTCTime[0] = '0';
                            this->Save_Data.UTCTime[1] = (this->Save_Data.UTCTime[1] % '9') + '0' - 1;
                        }
                        break; // 结束switch
                    case 2:
                        memcpy(usefullBuffer, subString, subStringNext - subString);
                        break; // 数据是否有效标志
                    case 3:
                        memcpy(this->Save_Data.latitude, subString, subStringNext - subString);
                        break; // 获取纬度信息
                    case 4:
                        memcpy(this->Save_Data.N_S, subString, subStringNext - subString);
                        break; // 获取N/S
                    case 5:
                        memcpy(this->Save_Data.longitude, subString, subStringNext - subString);
                        break; // 获取经度信息
                    case 6:
                        memcpy(this->Save_Data.E_W, subString, subStringNext - subString);
                        break; // 获取E/W
                    default:
                        break;
                    }
                    subString = subStringNext; // 下一个逗号位置给第一个指针，
                    this->Save_Data.isParseData = 1; // 手动给真值，（数据是否解析完成）
                }
                else
                {
                    while (ls_system_running.load())
                        ; // 解析错误
                }
            }
        }
    }
}

/********************************************************************************
 * @brief    拆分原始经纬度字符串，分离整数与小数部分
 * @param    Lon_Z：输出经度整数部分，Lon_X：输出经度小数部分
 * @param    Lat_Z：输出纬度整数部分，Lat_X：输出纬度小数部分
 * @return   char：1拆分成功，0字符串无小数点解析失败
 * @note     使用sscanf按小数点分割数值
 ********************************************************************************/
char lq_bd_gps::get_int_data(int *Lon_Z, int *Lon_X, int *Lat_Z, int *Lat_X)
{
    // 判断纬度值中是否有‘.'有:说明有数据 例如：3946.99715  变化范围在后六位中变化，最后一位不稳定可舍去。取值为3946和99715
    if ((strstr(this->Save_Data.latitude, ".")) != NULL)
    {
        sscanf(this->Save_Data.latitude, "%d.%d", &(*Lat_Z), &(*Lat_X));
    }
    else
        return 0;
    // 经度原理同上 例如：11628.32198  取值为11628 和 32198
    if ((strstr(this->Save_Data.longitude, ".")) != NULL)
    {
        sscanf(this->Save_Data.longitude, "%d.%d", &(*Lon_Z), &(*Lon_X));
    }
    else
        return 0;
    return 1;
}

/********************************************************************************
 * @brief   strtod把NMEA字符串转为浮点，÷100换算成十进制度
 * @param   Lon : 输出十进制经度(°)
 * @param   Lat : 输出十进制纬度(°)
 * @return  char:1成功 0原始坐标为0无效
 * @note    NMEA ddmm.xxxx → dd.mmxxxx(°)，除以100完成换算
 ********************************************************************************/
char lq_bd_gps::get_double_data(double *Lon, double *Lat)
{
    double W, J;
    J = strtod(this->Save_Data.longitude, NULL);
    W = strtod(this->Save_Data.latitude, NULL);
    if (W == 0.0)
        return 0;
    else
        *Lon = J / 100.0;
    if (J == 0.0)
        return 0;
    else
        *Lat = W / 100.0;
    return 1;
}

/********************************************************************************
 * @brief    串口单字节接收回调，拼接NMEA一帧数据
 * @param    data：串口单字节接收数据
 * @return   无
 * @note     非回车换行存入缓存；收到\r\n整帧转交GPS_Buffer并置接收标志
 ********************************************************************************/
void lq_bd_gps::read_data(const uint8_t data)
{
    static int num1;

    if (data == '\n' || data == '\r')
    {
        strcpy((char *)(this->Save_Data.GPS_Buffer), (const char *)this->read_buffer);
        this->Save_Data.isGetData = 1;
        num1 = 0;
    }
    else
    {
        this->read_buffer[num1++] = data;
    }
    // printf("%c", data);
}

/********************************************************************************
 * @brief    获取解析后的UTC时间字符串
 * @param    无
 * @return   const char*：UTC时间只读指针
 * @note     返回常量指针，外部不可修改内部原始时间缓存
 ********************************************************************************/
const char *lq_bd_gps::get_utc_time(void)
{
    return this->Save_Data.UTCTime;
}
