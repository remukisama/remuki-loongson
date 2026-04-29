#include "lq_reg_quadrature_encoder.hpp"
#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>

#define ENC_CMD_RST_CNT _IO('E', 1)
#define ENC_CMD_GET_DIR _IOR('E', 2, int)

/********************************************************************************
 * @brief   正交编码器的构造函数 (基于内核模块驱动)
 * @param   dev_id : 对应的内核字符设备索引(例如: 0为/dev/lq_encoder0，1为/dev/lq_encoder1)
 * @return  none.
 * @note    该应用层不再轮询，仅通过文件描述符读取内核统计的计数值
 ********************************************************************************/
ls_quadrature_encoder::ls_quadrature_encoder(uint8_t dev_id)
    : fd(-1), last_count(0), last_time_us(0)
{
    char dev_path[64];
    snprintf(dev_path, sizeof(dev_path), "/dev/lq_encoder%d", dev_id);
    fd = open(dev_path, O_RDWR);
    if (fd < 0) {
        std::cerr << "ls_quadrature_encoder error: Failed to open " << dev_path << ". "
                  << "Please ensure lq_encoder_irq.ko is loaded via 'insmod lq_encoder_irq.ko pinA=88,67 pinB=89,75'!" << std::endl;
    }
}

/********************************************************************************
 * @brief   析构函数
 * @param   none.
 * @return  none.
 ********************************************************************************/
ls_quadrature_encoder::~ls_quadrature_encoder()
{
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

/********************************************************************************
 * @brief   获取编码器增量计数
 * @param   none.
 * @return  float 与上次调用相比的脉冲增量(Δcount), 首次调用返回0
 ********************************************************************************/
float ls_quadrature_encoder::encoder_get_count(void)
{
    if (fd < 0) return 0.0f;

    long long current_count = 0;
    if (read(fd, &current_count, sizeof(current_count)) < 0) {
        return 0.0f;
    }

    // 首次调用仅建立基线, 避免产生异常大增量
    if (last_time_us == 0) {
        last_count = current_count;
        last_time_us = 1;
        return 0.0f;
    }

    float delta_count = (float)(current_count - last_count);
    last_count = current_count;

    return delta_count;
}

/********************************************************************************
 * @brief   获取编码器实时方向
 * @param   none.
 * @return  int8_t 1代表正转，-1代表反转，0代表停止/未知
 ********************************************************************************/
int8_t ls_quadrature_encoder::encoder_get_direction(void)
{
    if (fd < 0) return 0;
    int dir = 0;
    if (ioctl(fd, ENC_CMD_GET_DIR, &dir) == 0) {
        return (int8_t)dir;
    }
    return 0;
}

/********************************************************************************
 * @brief   重置编码器计数器
 * @param   none.
 * @return  none.
 ********************************************************************************/
void ls_quadrature_encoder::encoder_reset_counter(void)
{
    if (fd >= 0) {
        ioctl(fd, ENC_CMD_RST_CNT, NULL);
    }
}

