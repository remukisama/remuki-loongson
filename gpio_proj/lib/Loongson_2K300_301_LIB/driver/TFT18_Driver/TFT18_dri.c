#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/coda.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/mm.h>

#include "TFT18_dri.h"
#include "hw_gpio.h"

// 可修改区域
#define INIT_COLOR u16BLUE      // 初始化屏幕颜色
static int dc_gpio = 48;        // D/C 引脚号(命令/数据选择)
static int rst_gpio = 49;       // RST 引脚号(复位)
static int cs_gpio = 63;        // CS  引脚号(要拉低)

// 全局变量
static struct class *spi_class;     // 设备类
static struct cdev spi_cdev;        // 字符设备类
static struct spi_device *tft_spi;  // SPI 从设备类
static dev_t dev_id;                // 设备号
// 相关引脚变量
struct HardwareGPIO dc_Gpio;    // D/C 引脚自定义结构体
struct HardwareGPIO rst_Gpio;   // RST 引脚自定义结构体
struct HardwareGPIO cs_Gpio;    // CS  引脚自定义结构体
// 定义全局自旋锁
static spinlock_t dc_lock;
// 内核虚拟地址(帧缓冲)
uint16_t *fb_virt;
// 保存分配空间时的 order
static unsigned int alloc_order;
// 确认当前屏幕是横屏还是竖屏显示
static int display_status;
// 帧刷新使用的rxbuf
static uint8_t *rxbuf;

// 文件操作结构体
struct file_operations spi_fops = {
    .owner = THIS_MODULE,
    .open = tft_spi_open,
    .release = tft_spi_release,
    .unlocked_ioctl = tft_spi_ioctl,
    .read = tft_spi_read,
    .mmap = tft_spi_mmap,
};

// 匹配表，用于匹配设备
static const struct of_device_id spi_of_match[] = {
    { .compatible = DEV_ID_NAME },
    { /* 空 */ }
};
MODULE_DEVICE_TABLE(of, spi_of_match);

/*!
 * @brief   探测函数
 * @param   spi : SPI 设备
 * @return  0
 * @note    探测 SPI 设备时调用
 * @date    2025/4/28
 */
static int spi_probe(struct spi_device *spi)
{
    int ret;
    struct device *spi_dev;
    // 保存 SPI 设备指针
    tft_spi = spi;
    // 设置 SPI 模式
    tft_spi->mode = SPI_MODE_0;
    tft_spi->bits_per_word = 8;
    spi_setup(tft_spi);
    // 分配字符设备号
    ret = alloc_chrdev_region(&dev_id, 0, 1, DEV_ID_NAME);
    if (ret)
    {
        dev_err(&spi->dev, "Failed to alloc char device region\n");
        return ret;
    }
    // 创建设备类
    spi_class = class_create(THIS_MODULE, DEV_ID_NAME);
    if (IS_ERR(spi_class))
    {
        ret = PTR_ERR(spi_class);
        goto err_unreg_chrdev;
    }
    // 创建设备节点
    spi_dev = device_create(spi_class, &spi->dev, dev_id, NULL, DEV_FILE_NAME);
    if (IS_ERR(spi_dev)) {
        ret = PTR_ERR(spi_dev);
        goto err_destroy_class;
    }
    // 初始化 cdev
    cdev_init(&spi_cdev, &spi_fops);
    spi_cdev.owner = THIS_MODULE;
    ret = cdev_add(&spi_cdev, dev_id, 1);
    if (ret < 0) {
        goto err_destroy_device;
    }
    // GPIO 初始化
    ret = my_gpio_init(dc_gpio, &dc_Gpio);
    if (ret != 0)
    {
        dev_err(&spi->dev, "GPIO%d initialization failed (ret=%d)\n", dc_gpio, ret);
        goto err_destroy_device;
    }
    if (my_gpio_mode(GPIO_Mode_Out, &dc_Gpio) >= 0)
        dev_info(&spi->dev, "Obtained GPIO48 successfully\n");
    ret = my_gpio_init(rst_gpio, &rst_Gpio);
    if (ret != 0) {
        dev_err(&spi->dev, "GPIO%d initialization failed (ret=%d)\n", rst_gpio, ret);
        goto err_dc_gpio;
    }
    if (my_gpio_mode(GPIO_Mode_Out, &rst_Gpio) >= 0)
        dev_info(&spi->dev, "Obtained GPIO49 successfully\n");
    ret = my_gpio_init(cs_gpio, &cs_Gpio);
    if (ret != 0) {
        dev_err(&spi->dev, "GPIO%d initialization failed (ret=%d)\n", cs_gpio, ret);
        goto err_rst_gpio;
    }
    if (my_gpio_mode(GPIO_Mode_Out, &cs_Gpio) >= 0)
        dev_info(&spi->dev, "Obtained GPIO63 successfully\n");
    // 分配帧缓冲(使用 vmalloc 分配虚拟连续地址，适合小尺寸屏幕)
    alloc_order = get_order(FB_SIZE);
    fb_virt = (uint16_t*)__get_free_pages(GFP_KERNEL, alloc_order);
    if (!fb_virt)
    {
        dev_err(&spi->dev, "alloc frame buffer failed\n");
        goto err_cs_gpio;
    }
    printk("PAGE_SIZE = %ld, FB_SIZE = %lu\n", PAGE_SIZE, FB_SIZE);
    // 初始化屏幕颜色
    memset16(fb_virt, __builtin_bswap16(INIT_COLOR), FB_SIZE / sizeof(uint16_t));
    rxbuf = kmalloc(TFT18H * TFT18W * sizeof(uint16_t), GFP_KERNEL);
    if (!rxbuf)
    {
        dev_err(&spi->dev, "rxbuf allocation failed\n");
        goto err_kmalloc;
    }
    // 初始化自旋锁
    spin_lock_init(&dc_lock);
    dev_info(&spi->dev, "TFT driver probed successfully\n");
    return 0;
err_kmalloc:
    if (fb_virt)
    {
        // 释放内存（使用分配地址时的地址和 order）
        free_pages((unsigned long)fb_virt, alloc_order);
        printk(KERN_INFO "Freed frame buffer at address %p\n", fb_virt);
        fb_virt = NULL;
    }
err_cs_gpio:
    my_gpio_release(&dc_Gpio);
err_rst_gpio:
    my_gpio_release(&rst_Gpio);
err_dc_gpio:
    my_gpio_release(&cs_Gpio);
err_destroy_device:
    device_destroy(spi_class, dev_id);
err_destroy_class:
    class_destroy(spi_class);
err_unreg_chrdev:
    unregister_chrdev_region(dev_id, 1);
    dev_err(&spi->dev, "probe failed\n");
    return ret;
}

/*!
 * @brief   移除函数
 * @param   spi : SPI 设备
 * @return  0
 * @note    移除 SPI 设备时调用
 * @date    2025/4/28
 */
static int spi_remove(struct spi_device *spi)
{
    // 释放 SPI 设备
    cdev_del(&spi_cdev);
    // 释放设备节点
    device_destroy(spi_class, dev_id);
    // 释放设备类
    class_destroy(spi_class);
    // 释放字符设备号
    unregister_chrdev_region(dev_id, 1);
    // 释放 GPIO
    my_gpio_release(&dc_Gpio);
    my_gpio_release(&rst_Gpio);
    my_gpio_release(&cs_Gpio);
    // 释放帧缓冲区
    if (fb_virt)
    {
        // 释放内存（使用分配地址时的地址和 order）
        free_pages((unsigned long)fb_virt, alloc_order);
        printk(KERN_INFO "Freed frame buffer at address %p\n", fb_virt);
        fb_virt = NULL;
    }
    // 释放 rxbuf
    kfree(rxbuf);
    rxbuf = NULL;

    dev_info(&spi->dev, "SPI device removed\n");
    return 0;
}

// SPI 驱动结构体
static struct spi_driver spi_dev_driver = {
    .driver = {
        .name = DEV_ID_NAME,
        .of_match_table = spi_of_match,
    },
    .probe = spi_probe,     // 匹配成功时调用的函数
    .remove = spi_remove,   // 移除时调用的函数
};

/*!
 * @brief   模块初始化函数
 * @return  注册成功返回 0，失败返回错误码
 * @note    无
 * @date    2025/4/28
 */
static int __init spi_driver_init(void)
{
    return spi_register_driver(&spi_dev_driver);
}

/*!
 * @brief   模块退出函数
 * @return  无
 * @note    无
 * @date    2025/4/28
 */
static void __exit spi_driver_exit(void)
{
    spi_unregister_driver(&spi_dev_driver);
}

/*************************************************************************************************/
/**************************************** 文件操作相关函数 ****************************************/
/*************************************************************************************************/
/*!
 * @brief   打开文件操作
 * @param   node : 节点
 * @param   filp : 文件指针
 * @return  0
 * @note    打开文件时调用
 * @date    2025/4/28
 */
static int tft_spi_open(struct inode *node, struct file *filp)
{   
    filp->private_data = tft_spi;
    // 刷新屏幕颜色
    memset16(fb_virt, __builtin_bswap16(INIT_COLOR), FB_SIZE / sizeof(uint16_t));
    return 0;
}

/*!
 * @brief   关闭文件操作
 * @param   node : 节点
 * @param   filp : 文件指针
 * @return  0
 * @note    关闭文件时调用
 * @date    2025/4/28
 */
static int tft_spi_release(struct inode *node, struct file *filp)
{
    filp->private_data = tft_spi;
    // 刷新屏幕颜色
    memset16(fb_virt, __builtin_bswap16(INIT_COLOR), FB_SIZE / sizeof(uint16_t));
    return 0;
};

/*!
 * @brief   文件控制操作
 * @param   filp : 文件指针
 * @param   cmd : 命令
 * @param   arg : 参数
 * @return  成功返回 0，失败返回 -1
 * @note    控制文件时调用, 用于控制 TFT 屏幕的刷新、横屏/竖屏初始化等操作
 * @date    2025/4/28
 */
static long tft_spi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    switch (cmd)
    {
        case IOCTL_TFT_FLUSH:
            // 将帧缓冲数据通过 SPI 发送到 TFT 屏幕
            TFTSPI_Flush_Frame();return 0;
        case IOCTL_TFT_L_INIT:
            // 横屏初始化
            display_status = 0;
            TFTSPI_Init(0);
            TFTSPI_Flush_Frame();return 0;
        case IOCTL_TFT_V_INIT:
            // 竖屏初始化
            display_status = 1;
            TFTSPI_Init(1);
            TFTSPI_Flush_Frame();return 0;
        default:
            return -ENOTTY; // 未知命令
    }
}

/*!
 * @brief   文件读取操作
 * @param   filp  : 文件指针
 * @param   buf   : 缓冲区
 * @param   count : 读取字节数
 * @param   off   : 偏移量
 * @return  成功返回读取字节数，失败返回 -1
 * @note    读取文件时调用, 用于获取 TFT 屏幕的像素信息
 * @date    2025/4/28
 */
static ssize_t tft_spi_read(struct file *filp, char __user *buf, size_t count, loff_t *off)
{
    struct Pixel {
        uint8_t tft18_w;
        uint8_t tft18_h; 
    };
    struct Pixel pix;
    switch (display_status)
    {
        case 0:pix.tft18_w = TFT18W;pix.tft18_h = TFT18H;break;
        case 1:pix.tft18_w = TFT18H;pix.tft18_h = TFT18W;break;
        default:
            printk("Failed to obtain pixels\n");
            break;
    }
    // 检查用户空间缓冲区是否足够容纳结构体
    if (count < sizeof(pix))
        return -EINVAL;
    // 检查用户空间是否有效(防止野指针)
    if (!access_ok(VERIFY_WRITE, buf, sizeof(pix)))
        return -EFAULT;
    // 将内核结构体复制到用户空间缓冲区
    if (copy_to_user(buf, &pix, sizeof(pix)))
        return -EFAULT;
    return sizeof(pix);
}

/*!
 * @brief   缓冲区映射操作
 * @param   filp  : 文件指针
 * @param   vma   : 虚拟内存区域
 * @return  0
 * @note    映射文件时调用, 用于将内核空间的帧缓冲区映射到用户空间
 * @note    当上层调用 munmap 或者关闭文件描述符时会自动取消映射
 * @date    2025/4/28
 */
static int tft_spi_mmap(struct file *filp, struct vm_area_struct *vma)
{
    unsigned long pfn, offset, size;
    unsigned long page_size = PAGE_SIZE;
    // 计算用户空间请求的偏移量(字节)
    offset = vma->vm_pgoff << PAGE_SHIFT;
    size = vma->vm_end - vma->vm_start;
    // 检查映射大小和偏移是否合法
    if (size > FB_SIZE || offset + size > FB_SIZE)
    {
        dev_err(&tft_spi->dev, "mmap size/offset invalid: size=%lu, offset=%lu, FB_SIZE=%lu\n",
            size, offset, FB_SIZE);
        return -EINVAL;
    }
    // 检查偏移量和大小是否页对齐（remap_pfn_range 要求）
    if (offset % page_size != 0) {
        dev_err(&tft_spi->dev, "offset %lu not page-aligned (page_size=%lu)\n",
                offset, page_size);
        return -EINVAL;
    }
    if (size % page_size != 0) {
        dev_err(&tft_spi->dev, "size %lu not page-aligned (page_size=%lu)\n",
                size, page_size);
        return -EINVAL;
    } 
    // 获取内核虚拟地址对应的物理页帧号（__get_free_pages 分配的内存适用）
    pfn = virt_to_pfn((unsigned long)((char*)fb_virt + offset));
    if (!pfn)
    {
        dev_err(&tft_spi->dev, "vmalloc_to_pfn failed\n");
        return -EIO;
    }
    // 映射物理页到用户空间（权限继承自 vma->vm_page_prot）
    if (remap_pfn_range(vma,
                        vma->vm_start,      // 用户空间映射起始地址
                        pfn,                // 物理页帧号（从 offset 开始）
                        size,               // 映射大小
                        vma->vm_page_prot)) // 内存保护权限（读写/只读等）
    {
        dev_err(&tft_spi->dev, "remap_pfn_range failed\n");
        return -EIO;
    }
    return 0;
}

/*************************************************************************************************/
/**************************************** 屏幕处理相关函数 ****************************************/
/*************************************************************************************************/
/*!
 * @brief   缓冲区刷新指令
 * @param   无
 * @return  无
 * @note    发送刷新指令，刷新屏幕显示
 * @date    2025/4/28
 */
void TFTSPI_Flush_Frame(void)
{
    int ret;
    unsigned long flags;
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = (uint8_t*)fb_virt,
        .len = TFT18H * TFT18W * sizeof(uint16_t),
        .rx_buf = rxbuf,
        .bits_per_word = 8,
    };
    spin_lock_irqsave(&dc_lock, flags);
    my_gpio_set_value(0, &cs_Gpio);
    my_gpio_set_value(1, &dc_Gpio);
    spin_unlock_irqrestore(&dc_lock, flags);
    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(tft_spi, &msg);
    spin_lock_irqsave(&dc_lock, flags);
    my_gpio_set_value(0, &dc_Gpio);
    my_gpio_set_value(1, &cs_Gpio);
    spin_unlock_irqrestore(&dc_lock, flags);
}

/*!
 * @brief    TFT18初始化
 * @param    type ： 0:横屏  1：竖屏
 * @return   无
 * @note     如果修改管脚 需要修改初始化的管脚
 * @see      TFTSPI_Init(1);
 * @date     2025/4/28
 */
void TFTSPI_Init(uint8_t type)
{
    my_gpio_set_value(0, &rst_Gpio);
    msleep(120);
    my_gpio_set_value(1, &rst_Gpio);
    msleep(50);
    TFTSPI_Write_Cmd(0x11); // 关闭睡眠，振荡器工作
    msleep(50);
    TFTSPI_Write_Cmd(0x3a); // 每次传送16位数据(VIPF3-0=0101)，每个像素16位(IFPF2-0=101)
    TFTSPI_Write_Byte(0x55);
    TFTSPI_Write_Cmd(0x26);
    TFTSPI_Write_Byte(0x04);
    TFTSPI_Write_Cmd(0xf2); // Driver Output Control(1)
    TFTSPI_Write_Byte(0x01);
    TFTSPI_Write_Cmd(0xe0); // Driver Output Control(1)
    TFTSPI_Write_Byte(0x3f);
    TFTSPI_Write_Byte(0x25);
    TFTSPI_Write_Byte(0x1c);
    TFTSPI_Write_Byte(0x1e);
    TFTSPI_Write_Byte(0x20);
    TFTSPI_Write_Byte(0x12);
    TFTSPI_Write_Byte(0x2a);
    TFTSPI_Write_Byte(0x90);
    TFTSPI_Write_Byte(0x24);
    TFTSPI_Write_Byte(0x11);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Cmd(0xe1); // Driver Output Control(1)
    TFTSPI_Write_Byte(0x20);
    TFTSPI_Write_Byte(0x20);
    TFTSPI_Write_Byte(0x20);
    TFTSPI_Write_Byte(0x20);
    TFTSPI_Write_Byte(0x05);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x15);
    TFTSPI_Write_Byte(0xa7);
    TFTSPI_Write_Byte(0x3d);
    TFTSPI_Write_Byte(0x18);
    TFTSPI_Write_Byte(0x25);
    TFTSPI_Write_Byte(0x2a);
    TFTSPI_Write_Byte(0x2b);
    TFTSPI_Write_Byte(0x2b);
    TFTSPI_Write_Byte(0x3a);
    TFTSPI_Write_Cmd(0xb1);  // 0xb1      	//设置屏幕刷新频率
    TFTSPI_Write_Byte(0x00); // 0x08		//DIVA=8
    TFTSPI_Write_Byte(0x00); // 0x08		//VPA =8，约90Hz
    TFTSPI_Write_Cmd(0xb4);  // LCD Driveing control
    TFTSPI_Write_Byte(0x07); // NLA=1,NLB=1,NLC=1
    TFTSPI_Write_Cmd(0xc0);  // LCD Driveing control  Power_Control1
    TFTSPI_Write_Byte(0x0a);
    TFTSPI_Write_Byte(0x02);
    TFTSPI_Write_Cmd(0xc1); // LCD Driveing control
    TFTSPI_Write_Byte(0x02);
    TFTSPI_Write_Cmd(0xc5); // LCD Driveing control
    TFTSPI_Write_Byte(0x4f);
    TFTSPI_Write_Byte(0x5a);
    TFTSPI_Write_Cmd(0xc7); // LCD Driveing control
    TFTSPI_Write_Byte(0x40);
    TFTSPI_Write_Cmd(0x2a);  // 配置MCU可操作的LCD内部RAM横坐标起始、结束参数
    TFTSPI_Write_Byte(0x00); // 横坐标起始地址0x0000
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00); // 横坐标结束地址0x007f(127)
    TFTSPI_Write_Byte(0xa8); // 7f
    TFTSPI_Write_Cmd(0x2b);  // 配置MCU可操作的LCD内部RAM纵坐标起始结束参数
    TFTSPI_Write_Byte(0x00); // 纵坐标起始地址0x0000
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00); // 纵坐标结束地址0x009f(159)
    TFTSPI_Write_Byte(0xb3); // 9f
    TFTSPI_Write_Cmd(0x36);  // 配置MPU和DDRAM对应关系
    if (type)
        TFTSPI_Write_Byte(0xC0); // 竖屏显示          //MX=1,MY=1
    else
        TFTSPI_Write_Byte(0xA0); // 横屏显示

    TFTSPI_Write_Cmd(0xb7);  // LCD Driveing control
    TFTSPI_Write_Byte(0x00); // CRL=0
    TFTSPI_Write_Cmd(0x29);  // 开启屏幕显示
    TFTSPI_Write_Cmd(0x2c);  // 设置为LCD接收数据/命令模式
}

/*!
 * @brief    写入命令
 * @param    cmd : 命令
 * @return   无
 * @note     内部调用
 * @see      TFTSPI_Write_Cmd(0x2c);
 * @date     2025/4/28
 */
void TFTSPI_Write_Cmd(uint8_t cmd)
{
    int ret;
    unsigned long flags;
    static uint8_t txbuf[1], rxbuf[1];
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = txbuf,
        .len = 1,
        .rx_buf = rxbuf,
        .bits_per_word = 8,
    };
    spin_lock_irqsave(&dc_lock, flags);
    my_gpio_set_value(0, &cs_Gpio);
    my_gpio_set_value(0, &dc_Gpio);
    txbuf[0] = cmd;
    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(tft_spi, &msg);
    if (ret)
        printk(KERN_ERR "SPI transfer : 0x%x, failed: %d\n", txbuf[0], ret);
    my_gpio_set_value(1, &dc_Gpio);
    my_gpio_set_value(1, &cs_Gpio);
    spin_unlock_irqrestore(&dc_lock, flags);
}

/*!
 * @brief    写入字节
 * @param    data : 字节数据
 * @return   无
 * @note     内部调用
 * @see      TFTSPI_Write_Byte(0x00);
 * @date     2025/4/28
 */
void TFTSPI_Write_Byte(uint8_t data)
{
    int ret;
    unsigned long flags;
    static uint8_t txbuf[1], rxbuf[1];
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = txbuf,
        .len = 1,
        .rx_buf = rxbuf,
        .bits_per_word = 8,
    };
    spin_lock_irqsave(&dc_lock, flags);
    my_gpio_set_value(0, &cs_Gpio);
    my_gpio_set_value(1, &dc_Gpio);
    txbuf[0] = data;
    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(tft_spi, &msg);
    if (ret)
        printk(KERN_ERR "SPI transfer : 0x%x, failed: %d\n", txbuf[0], ret);
    my_gpio_set_value(0, &dc_Gpio);
    my_gpio_set_value(1, &cs_Gpio);
    spin_unlock_irqrestore(&dc_lock, flags);
}

/*!
 * @brief    写入半字
 * @param    data : 半字数据
 * @return   无
 * @note     内部调用
 * @see      TFTSPI_Write_Word(0x0000);
 * @date     2025/4/28
 */
void TFTSPI_Write_Word(uint16_t data)
{
    int ret;
    unsigned long flags;
    static uint8_t txbuf[2], rxbuf[2];
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = txbuf,
        .len = 2,
        .rx_buf = rxbuf,
        .bits_per_word = 8,
    };
    spin_lock_irqsave(&dc_lock, flags);
    my_gpio_set_value(0, &cs_Gpio);
    my_gpio_set_value(1, &dc_Gpio);
    txbuf[0] = data >> 8;
    txbuf[1] = data & 0xFF;
    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(tft_spi, &msg);
    if (ret)
        printk(KERN_ERR "SPI transfer : 0x%x, failed: %d\n", (txbuf[0] >> 8) | (txbuf[1]), ret);
    my_gpio_set_value(0, &dc_Gpio);
    my_gpio_set_value(1, &cs_Gpio);
    spin_unlock_irqrestore(&dc_lock, flags);
}

/*!
 * @brief    重置地址
 * @param    无
 * @return   无
 * @note     内部调用
 * @see      TFTSPI_Addr_Rst();
 * @date     2025/4/28
 */
void TFTSPI_Addr_Rst(void)
{
    TFTSPI_Write_Cmd(0x2a);  // 配置MCU可操作的LCD内部RAM横坐标起始、结束参数
    TFTSPI_Write_Byte(0x00); // 横坐标起始地址0x0000
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00); // 横坐标结束地址0x007f(127)
    TFTSPI_Write_Byte(0xa8); // 7f
    TFTSPI_Write_Cmd(0x2b);  // 配置MCU可操作的LCD内部RAM纵坐标起始结束参数
    TFTSPI_Write_Byte(0x00); // 纵坐标起始地址0x0000
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00); // 纵坐标结束地址0x009f(159)
    TFTSPI_Write_Byte(0xb3); // 9f
    TFTSPI_Write_Cmd(0x2C);  // GRAM接收MCU数据或命令
}

/*************************************************************************************************/
/**************************************** GPIO相关操作函数 ****************************************/
/*************************************************************************************************/
/*!
 * @brief   初始化 GPIO
 * @param   gpio : GPIO 的引脚号
 * @param   Gpio : GPIO 的自定义类
 * @return  失败返回错误码，成功返回 0
 */
int my_gpio_init(uint8_t gpio, struct HardwareGPIO *Gpio)
{
    // 获取引脚号
    Gpio->gpio = gpio;
    // 获取基础寄存器映射地址
    Gpio->gpio_base = ioremap(LS_GPIO_BASE_ADDR, PAGESIZE);
    if (Gpio->gpio_base == NULL)
    {
        printk(KERN_ERR "gpio%d gpio_base ioremap failed\n", Gpio->gpio);
        return -ENOMEM;
    }
    // 获取复用寄存器映射地址
    Gpio->gpio_reuse = ioremap(LS_GPIO_REUSE_ADDR + (gpio / 16) * LS_GPIO_REUSE_OFS, PAGESIZE);
    if (Gpio->gpio_reuse == NULL)
    {
        printk(KERN_ERR "gpio%d gpio_reuse ioremap failed\n", Gpio->gpio);
        return -ENOMEM;
    }
    // 将对应引脚复用为 GPIO 模式
    writel(readl(Gpio->gpio_reuse) & ~(0b11 << (gpio % 16 * 2)), Gpio->gpio_reuse);
    return 0;
}

/*!
 * @brief   设置 GPIO 模式
 * @param   mode : GPIO 的模式。GPIO_Mode_In 为输入模式，GPIO_Mode_Out 为输出模式
 * @param   Gpio : GPIO 的自定义类
 * @return  失败返回 -1，成功返回 0
 */
int my_gpio_mode(uint8_t mode, struct HardwareGPIO *Gpio)
{
    switch (mode)
    {
        case GPIO_Mode_In:
            writel(readl(Gpio->gpio_base + LS_GPIO_OEN_OFFSET(Gpio->gpio)) | BIT(Gpio->gpio % 8), Gpio->gpio_base + LS_GPIO_OEN_OFFSET(Gpio->gpio));
            break;
        case GPIO_Mode_Out:
            writel(readl(Gpio->gpio_base + LS_GPIO_OEN_OFFSET(Gpio->gpio)) & ~BIT(Gpio->gpio % 8), Gpio->gpio_base + LS_GPIO_OEN_OFFSET(Gpio->gpio));
            break;
        default:
            printk(KERN_ERR "gpio%d mode configure failed\n", Gpio->gpio);
            return -1;
            break;
    }
    return 0;
}

/*!
 * @brief   设置 GPIO 高低电平
 * @param   val  : GPIO 的电平值
 * @param   Gpio : GPIO 的自定义类
 * @return  失败返回 -1，成功返回设置的电平值
 */
int my_gpio_set_value(uint8_t val, struct HardwareGPIO *Gpio)
{
    switch (val)
    {
        case 1:
            writel(readl(Gpio->gpio_base + LS_GPIO_O_OFFSET(Gpio->gpio)) | BIT(Gpio->gpio % 8), Gpio->gpio_base + LS_GPIO_O_OFFSET(Gpio->gpio));
            break;
        case 0:
            writel(readl(Gpio->gpio_base + LS_GPIO_O_OFFSET(Gpio->gpio)) & ~BIT(Gpio->gpio % 8), Gpio->gpio_base + LS_GPIO_O_OFFSET(Gpio->gpio));
            break;
        default:
            printk(KERN_ERR "gpio%d value setting failed\n", Gpio->gpio);
            return -1;
            break;
    }
    return val;
}

/*!
 * @brief   获取 GPIO 高低电平
 * @param   Gpio : GPIO 的自定义类
 * @return  返回获取的电平值
 */
int my_gpio_get_value(struct HardwareGPIO *Gpio)
{
    return ((readl(Gpio->gpio_base + LS_GPIO_I_OFFSET(Gpio->gpio % 8)) & BIT(Gpio->gpio % 8)) == BIT(Gpio->gpio % 8));
}

/*!
 * @brief   清理 GPIO
 * @param   GPIO : GPIO 的自定义类
 * @return  无
 */
void my_gpio_release(struct HardwareGPIO *Gpio)
{
    iounmap(Gpio->gpio_base);
    iounmap(Gpio->gpio_reuse);   
}

// 加载模块初始化和卸载函数
module_init(spi_driver_init);
module_exit(spi_driver_exit);

MODULE_AUTHOR("LQ_012");
MODULE_DESCRIPTION("1.8-inch TFT-SPI Display Driver Terminal");
MODULE_LICENSE("GPL");
