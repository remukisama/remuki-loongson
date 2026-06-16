#ifndef __LQ_DISPLAY_TFT18_HPP
#define __LQ_DISPLAY_TFT18_HPP 

#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/ioctl.h>
#include "lq_display_font.hpp"
#include "lq_display_types.hpp"

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

/* 文件名称 */
#define TFT18_DEV_NAME      ( "/dev/LQ_TFT_1.8" )

/* 1.8寸 TFT 屏幕尺寸定义 */
#define TFT18_WIDTH         ( 162 )
#define TFT18_HEIGHT        ( 132 )

/* 帧缓冲大小 */
#define TFT18_PageSize      ( 16384 )
#define TFT18_FB_SIZE       ( TFT18_PageSize * 3 )

#define TFT18_SSIZE         ( TFT18_WIDTH * TFT18_HEIGHT )

/* 1.8寸 TFT 屏幕相关幻数号 */
#define IOCTL_TFT18_MAGIC   't'         // 自定义幻数号, 用于区分不同的设备驱动
#define IOCTL_TFT18_FLUSH   _IO('t', 1) // 刷新屏幕
#define IOCTL_TFT18_L_INIT  _IO('t', 2) // 衡屏初始化
#define IOCTL_TFT18_V_INIT  _IO('t', 3) // 竖屏初始化

/****************************************************************************************************
 * @brief   函数定义
 ****************************************************************************************************/

void lq_tft18_drv_init(uint8_t type);       // 屏幕初始化

void lq_tft18_drv_cls       (lq_display_color_t color_dat);                                                   // 全屏显示单色画面
void lq_tft18_drv_fill_area (uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, lq_display_color_t color_dat);   // 填充指定区域

void lq_tft18_drv_draw_dot      (uint8_t x,  uint8_t y,  lq_display_color_t color);                               // 画点
void lq_tft18_drv_draw_line     (uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, lq_display_color_t color);       // 画线
void lq_tft18_drv_draw_rectangle(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, lq_display_color_t color_dat);   // 画矩形边框
void lq_tft18_drv_draw_circle   (uint8_t x , uint8_t y , uint8_t r , lq_display_color_t color_dat);               // 画圆

void lq_tft18_drv_p6x8_str (uint8_t x, uint8_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color);     // 显示P6X8字符
void lq_tft18_drv_p8x8_str (uint8_t x, uint8_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color);     // 显示P8X8字符
void lq_tft18_drv_p8x16_str(uint8_t x, uint8_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color);     // 显示P8X16字符

void lq_tft18_drv_p16x16_cstr(uint8_t x, uint8_t y, const char *s_dat, lq_display_color_t word_color, lq_display_color_t back_color);   // 显示P16X16汉字

void lq_tft18_drv_road   (uint8_t wide_start, uint8_t high_start, uint8_t high, uint8_t wide, uint8_t *Pixle); // 显示灰度图
void lq_tft18_drv_binroad(uint8_t wide_start, uint8_t high_start, uint8_t high, uint8_t wide, uint8_t *Pixle); // 显示二值图

#ifdef LQ_HAVE_OPENCV
#include <opencv2/opencv.hpp>
void lq_tft18_drv_road_color(uint8_t wide_start, uint8_t high_start, const cv::Mat &Pixle);  // 绘制彩色图像
#endif

void lq_tft18_drv_flush();  // 刷新屏幕

#endif
