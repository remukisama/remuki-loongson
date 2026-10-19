#ifndef __LQ_MPU6050_DRV_COMMON_H__
#define __LQ_MPU6050_DRV_COMMON_H__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

/******************************** mpu6050 底层驱动相关宏定义 ********************************/

#define DEVICE_ID_NAME          ( "ls,lq_i2c_mpu6050" ) /* 设备名称, 请勿修改 */
#define DETECT_INTERVAL_MS      ( 500 )                 /* 定义循环检测设备的时间间隔 (毫秒), 感觉间隔时间不合适, 可适当修改（0.5s） */
#define DEVICE_CNT              ( 1 )                   /* 设备数量, 请勿修改 */
#define MPU6050_DEV_ADDR        ( 0x68 )                /* MPU6050设备地址, 请勿修改 */
#define DEVICE_NAME             ( "lq_i2c_mpu6050" )    /* 设备文件名称, 请勿修改 */

/* MPU6050 相关幻数号, 请勿修改 */
#define I2C_MPU6050_MAGIC       ( 'i' )                         // 自定义幻数号，用于区分不同的设备驱动
#define I2C_GET_MPU6050_ID      ( _IO(I2C_MPU6050_MAGIC, 1) )   // 获取 MPU6050 ID
#define I2C_GET_MPU6050_TEM     ( _IO(I2C_MPU6050_MAGIC, 2) )   // 获取 MPU6050 温度
#define I2C_GET_MPU6050_ANG     ( _IO(I2C_MPU6050_MAGIC, 3) )   // 获取 MPU6050 角度值
#define I2C_GET_MPU6050_ACC     ( _IO(I2C_MPU6050_MAGIC, 4) )   // 获取 MPU6050 加速度
#define I2C_GET_MPU6050_GYRO    ( _IO(I2C_MPU6050_MAGIC, 5) )   // 获取 MPU6050 角度和加速度值

/**************************** 存储当前所支持的所有I2C设备地址列表 ****************************/

/* 自定义结构体, 存储了一些 iic 设备所需要的成员变量, 请勿修改 */
struct ls_i2c_dev {
    struct i2c_client  *client;     // 表示连接到 iic 总线上的客户端设备，包含了设备的各种信息和操作方式
    struct cdev         cdev;       // 表示字符设备的核心结构体，包含字符设备的各种属性和操作方法
    struct class       *class;      // 表示设备类的核心结构体
    struct device      *device;     // 表示系统中的一个具体设备
    struct i2c_adapter *adapter;    // 表示连接到 iic 总线上的适配器，包含了适配器的各种信息和操作方式
    dev_t               dev_id;     // 表示该设备的设备号-
};

/* 自定义结构体, 存储定时器相关信息, 请勿修改 */
struct ls_cycle_data
{
    struct work_struct work;            // 定时器工作队列结构体
    struct timer_list cycle_detection;  // 定时器结构体
    struct ls_i2c_dev *Dev;             // 设备结构体指针
    struct workqueue_struct *wq;       // 工作队列结构体指针
};

/************************************* 各模块初始化函数 *************************************/

/* 定义一个函数指针类型，指向一个函数，返回值是int，参数是struct ls_i2c_dev *dev, LQ_List *list类型的 */
typedef int (*DeviceInitFunc)(struct ls_i2c_dev *dev);

int  device_inspection              (struct ls_i2c_dev *dev, DeviceInitFunc deviceInit);/* 检查当前连接设备 */
int  device_init                    (struct ls_i2c_dev *dev);                           /* 初始化设备 */
void cycle_detection_timer_callback (struct timer_list *timer);                         /* 循环检测设备定时器回调函数 */

/************************************ 上层接口的函数声明 ************************************/

int     i2c_open    (struct inode *inode, struct file *f);                              /* 上层 open 函数相关实现 */
int     i2c_release (struct inode *inode, struct file *f);                              /* 上层 close 函数相关实现 */
ssize_t i2c_read    (struct file *f, char __user *buf, size_t cnt, loff_t *off);        /* 上层 read 函数相关实现 */
ssize_t i2c_write   (struct file *f, const char __user *buf, size_t cnt, loff_t *off);  /* 上层 write 函数相关实现 */
long    i2c_ioctl   (struct file *f, unsigned int cmd, unsigned long arg);              /* 上层 ioctl 函数相关实现 */

/********************************** 设备驱动的相关函数声明 **********************************/

int i2c_probe (struct i2c_client *client, const struct i2c_device_id *id);   /* 设备驱动的探测函数 */
int i2c_remove(struct i2c_client *c);   

#endif