#ifndef __IPS_ST7789_DRI_H__
#define __IPS_ST7789_DRI_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

// 设备文件名称
#define DEV_FILE_NAME       ( "LQ_IPS_ST7789" )

// 设备匹配名称
#define DEV_ID_NAME         ( "loongson,ips-st7789-driver" )

/* ST7789 面板参数 */
#define IPSW                ( 240 )
#define IPSH                ( 320 )
#define IPS_LAND_W          ( 320 )
#define IPS_LAND_H          ( 240 )

/* 帧缓冲大小（与页对齐，便于 mmap） */
#define FB_SIZE             ( PAGE_SIZE * 10 )

/* 颜色宏 */
#define u16RED		        ( 0xf800 )      // 红色
#define u16GREEN	        ( 0x07e0 )      // 绿色
#define u16BLUE	            ( 0x001f )      // 蓝色
#define u16PURPLE	        ( 0xf81f )      // 紫色
#define u16YELLOW	        ( 0xffe0 )      // 黄色
#define u16CYAN	            ( 0x07ff ) 	    // 青蓝色
#define u16ORANGE	        ( 0xfc08 )      // 橙色
#define u16BLACK	        ( 0x0000 )      // 黑色
#define u16WHITE	        ( 0xffff )      // 白色

#define IOCTL_IPS20_MAGIC   'p'                         // 自定义幻数号, 用于区分不同的设备驱动
#define IOCTL_IPS_FLUSH     ( _IO(IOCTL_IPS20_MAGIC, 1) ) // 刷新命令
#define IOCTL_IPS_L_INIT    ( _IO(IOCTL_IPS20_MAGIC, 2) ) // 横屏初始化
#define IOCTL_IPS_V_INIT    ( _IO(IOCTL_IPS20_MAGIC, 3) ) // 竖屏初始化

/*************************************************************************************************/
/*************************************** 文件操作相关函数声明 **************************************/
/*************************************************************************************************/
static int     ips_spi_open     (struct inode *, struct file *);
static int     ips_spi_release  (struct inode *, struct file *);
static long    ips_spi_ioctl    (struct file  *, unsigned int, unsigned long);
static ssize_t ips_spi_read     (struct file  *, char __user *, size_t, loff_t *);
static int     ips_spi_mmap     (struct file  *, struct vm_area_struct *);

/*************************************************************************************************/
/*************************************** 数据传输相关函数声明 **************************************/
/*************************************************************************************************/

void IPSSPI_Write_Cmd   (uint8_t  cmd);     /* 写命令（D/C = 0） */
void IPSSPI_Write_Byte  (uint8_t  data);    /* 写字节（D/C = 1） */
void IPSSPI_Write_Word  (uint16_t data);    /* 写字（D/C = 1） */

/*************************************************************************************************/
/**************************************** 屏幕相关函数声明 ****************************************/
/*************************************************************************************************/

void IPSSPI_Init        (uint8_t type); /* 初始化显示屏 */

void IPSSPI_Flush_Frame (void);         /* 刷新函数 */
void IPSSPI_Addr_Rst    (void);         /* 复位地址窗口 */

#endif
