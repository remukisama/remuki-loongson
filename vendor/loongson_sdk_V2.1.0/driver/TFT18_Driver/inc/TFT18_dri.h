#ifndef __TFT18_DRI_H__
#define __TFT18_DRI_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

// 设备文件名称
#define DEV_FILE_NAME       ( "LQ_TFT_1.8" )

// 设备匹配名称
#define DEV_ID_NAME         ( "loongson,tft-driver" )

/* TFT 参数定义 */
#define TFT18W              ( 162 )
#define TFT18H              ( 132 )

/* 帧缓冲大小 */
#define FB_SIZE             ( PAGE_SIZE * 3 )

/* 颜色数据 */
#define u16RED		        ( 0xf800 )      // 红色
#define u16GREEN	        ( 0x07e0 )      // 绿色
#define u16BLUE	            ( 0x001f )      // 蓝色
#define u16PURPLE	        ( 0xf81f )      // 紫色
#define u16YELLOW	        ( 0xffe0 )      // 黄色
#define u16CYAN	            ( 0x07ff ) 	    // 蓝绿色
#define u16ORANGE	        ( 0xfc08 )      // 橙色
#define u16BLACK	        ( 0x0000 )      // 黑色
#define u16WHITE	        ( 0xffff )      // 白色

#define IOCTL_TFT_FLUSH     ( _IO('t', 1) ) // 刷新命令（幻数 't'，序号 1）
#define IOCTL_TFT_L_INIT    ( _IO('t', 2) ) // 横屏初始化
#define IOCTL_TFT_V_INIT    ( _IO('t', 3) ) // 竖屏初始化

/*************************************************************************************************/
/*************************************** 文件操作相关函数声明 **************************************/
/*************************************************************************************************/
static int     tft_spi_open     (struct inode *, struct file *);
static int     tft_spi_release  (struct inode *, struct file *);
static long    tft_spi_ioctl    (struct file  *, unsigned int, unsigned long);
static ssize_t tft_spi_read     (struct file  *, char __user *, size_t, loff_t *);
static int     tft_spi_mmap     (struct file  *, struct vm_area_struct *);

/*************************************************************************************************/
/*************************************** 数据发送相关函数声明 **************************************/
/*************************************************************************************************/

void TFTSPI_Write_Cmd   (uint8_t  cmd);     /* 写命令（D/C = 0） */
void TFTSPI_Write_Byte  (uint8_t  data);    /* 写字节（D/C = 1） */
void TFTSPI_Write_Word  (uint16_t data);    /* 写半字（D/C = 1） */

/*************************************************************************************************/
/**************************************** 屏幕相关函数声明 ****************************************/
/*************************************************************************************************/

void TFTSPI_Init        (uint8_t type); /* 初始化显示屏幕 */

void TFTSPI_Flush_Frame (void);         /* 刷新函数 */
void TFTSPI_Addr_Rst    (void);         /* 重置地址 */

#endif
