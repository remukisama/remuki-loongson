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

#include "IPS_ST7789_dri.h"
#include "hw_gpio.h"

// 可修改区域
#define INIT_COLOR u16BLUE     // 初始化屏幕颜色
static int dc_gpio = 48;        // D/C 引脚号(数据/命令选择)
static int rst_gpio = 49;       // RST 引脚号(复位)
static int cs_gpio = 63;        // CS  引脚号(片选)

// 全局变量
static struct class *spi_class;     // 设备类
static struct cdev spi_cdev;        // 字符设备
static struct spi_device *ips_spi;  // SPI 设备对象
static dev_t dev_id;                // 设备号
// 硬件引脚对象
struct HardwareGPIO dc_Gpio;    // D/C GPIO结构体
struct HardwareGPIO rst_Gpio;   // RST GPIO结构体
struct HardwareGPIO cs_Gpio;    // CS  GPIO结构体
// 自旋锁
static spinlock_t dc_lock;
// 内核虚拟地址(帧缓冲)
uint16_t *fb_virt;
// 分配页阶数 order
static unsigned int alloc_order;
// 当前显示方向
static int display_status;
// SPI 接收缓冲
static uint8_t *rxbuf;

static void ips_memset16(void *s, uint16_t c, size_t count)
{
    uint16_t *p = (uint16_t*)s;
    size_t i;
    for (i = 0; i < count; i++)
    {
        p[i] = c;
    }
}

// 文件操作结构体
struct file_operations spi_fops = {
    .owner = THIS_MODULE,
    .open = ips_spi_open,
    .release = ips_spi_release,
    .unlocked_ioctl = ips_spi_ioctl,
    .read = ips_spi_read,
    .mmap = ips_spi_mmap,
};

// 匹配表，用于匹配设备树
static const struct of_device_id spi_of_match[] = {
    { .compatible = DEV_ID_NAME },
    { /* 结束 */ }
};
MODULE_DEVICE_TABLE(of, spi_of_match);

/*!
 * @brief   探测函数
 * @param   spi : SPI 设备
 * @return  0
 * @note    探测 SPI 设备时调用
 * @date    2026/2/18
 */
static int spi_probe(struct spi_device *spi)
{
    int ret;
    struct device *spi_dev;
    // 保存 SPI 设备指针
    ips_spi = spi;
    // 设置 SPI 模式
    ips_spi->mode = SPI_MODE_0;
    ips_spi->bits_per_word = 8;
    spi_setup(ips_spi);
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
        goto err_destroy_cdev;
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

    // 分配帧缓冲
    alloc_order = get_order(FB_SIZE);
    fb_virt = (uint16_t*)__get_free_pages(GFP_KERNEL, alloc_order);
    if (!fb_virt)
    {
        dev_err(&spi->dev, "alloc frame buffer failed\n");
        ret = -ENOMEM;
        goto err_cs_gpio;
    }
    printk("PAGE_SIZE = %ld, FB_SIZE = %lu\n", PAGE_SIZE, FB_SIZE);
    // 初始化屏幕颜色
    ips_memset16(fb_virt, __builtin_bswap16(INIT_COLOR), (IPSW * IPSH));

    rxbuf = kmalloc(IPSH * IPSW * sizeof(uint16_t), GFP_KERNEL);
    if (!rxbuf)
    {
        dev_err(&spi->dev, "rxbuf allocation failed\n");
        ret = -ENOMEM;
        goto err_kmalloc;
    }
    // 初始化锁
    spin_lock_init(&dc_lock);
    dev_info(&spi->dev, "ST7789 IPS driver probed successfully\n");
    return 0;

err_kmalloc:
    if (fb_virt)
    {
        free_pages((unsigned long)fb_virt, alloc_order);
        printk(KERN_INFO "Freed frame buffer at address %p\n", fb_virt);
        fb_virt = NULL;
    }
err_cs_gpio:
    my_gpio_release(&cs_Gpio);
err_rst_gpio:
    my_gpio_release(&rst_Gpio);
err_dc_gpio:
    my_gpio_release(&dc_Gpio);
err_destroy_cdev:
    cdev_del(&spi_cdev);
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
 * @date    2026/2/18
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
    // 释放帧缓冲
    if (fb_virt)
    {
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
    .probe = spi_probe,     // 匹配成功时调用
    .remove = spi_remove,   // 移除时调用
};

/*!
 * @brief   模块初始化函数
 * @return  注册成功返回 0，失败返回错误码
 * @note    无
 * @date    2026/2/18
 */
static int __init spi_driver_init(void)
{
    return spi_register_driver(&spi_dev_driver);
}

/*!
 * @brief   模块退出函数
 * @return  无
 * @note    无
 * @date    2026/2/18
 */
static void __exit spi_driver_exit(void)
{
    spi_unregister_driver(&spi_dev_driver);
}

/*************************************************************************************************/
/**************************************** 文件操作相关函数 ****************************************/
/*************************************************************************************************/
/*!
 * @brief   打开文件函数
 * @param   node : 节点
 * @param   filp : 文件指针
 * @return  0
 * @note    打开文件时调用
 * @date    2026/2/18
 */
static int ips_spi_open(struct inode *node, struct file *filp)
{
    filp->private_data = ips_spi;
    // 刷新屏幕颜色
    ips_memset16(fb_virt, __builtin_bswap16(INIT_COLOR), (IPSW * IPSH));
    return 0;
}

/*!
 * @brief   关闭文件函数
 * @param   node : 节点
 * @param   filp : 文件指针
 * @return  0
 * @note    关闭文件时调用
 * @date    2026/2/18
 */
static int ips_spi_release(struct inode *node, struct file *filp)
{
    filp->private_data = ips_spi;
    // 刷新屏幕颜色
    ips_memset16(fb_virt, __builtin_bswap16(INIT_COLOR), (IPSW * IPSH));
    return 0;
};

/*!
 * @brief   文件控制操作
 * @param   filp : 文件指针
 * @param   cmd : 命令
 * @param   arg : 参数
 * @return  成功返回 0，失败返回 -1
 * @note    控制文件时调用, 用于控制 IPS 屏幕的刷新、横/竖初始化等操作
 * @date    2026/2/18
 */
static long ips_spi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    switch (cmd)
    {
        case IOCTL_IPS_FLUSH:
            IPSSPI_Flush_Frame();return 0;
        case IOCTL_IPS_L_INIT:
            display_status = 0;
            IPSSPI_Init(0);
            IPSSPI_Flush_Frame();return 0;
        case IOCTL_IPS_V_INIT:
            display_status = 1;
            IPSSPI_Init(1);
            IPSSPI_Flush_Frame();return 0;
        default:
            return -ENOTTY;
    }
}

/*!
 * @brief   文件读取函数
 * @param   filp  : 文件指针
 * @param   buf   : 缓冲区
 * @param   count : 读取字节数
 * @param   off   : 偏移量
 * @return  成功返回读取字节数，失败返回 -1
 * @note    读取文件时调用, 用于获取 IPS 屏幕分辨率信息
 * @date    2026/2/18
 */
static ssize_t ips_spi_read(struct file *filp, char __user *buf, size_t count, loff_t *off)
{
    struct Pixel {
        uint16_t ips_w;
        uint16_t ips_h;
    };
    struct Pixel pix;

    switch (display_status)
    {
        case 0:pix.ips_w = IPS_LAND_W;pix.ips_h = IPS_LAND_H;break;
        case 1:pix.ips_w = IPSW;pix.ips_h = IPSH;break;
        default:
            printk("Failed to obtain pixels\n");
            break;
    }
    // 判断用户空间缓冲区是否足够容纳结构体
    if (count < sizeof(pix))
        return -EINVAL;
    // 判断用户空间是否有效
    if (!access_ok(VERIFY_WRITE, buf, sizeof(pix)))
        return -EFAULT;
    // 复制到用户空间
    if (copy_to_user(buf, &pix, sizeof(pix)))
        return -EFAULT;
    return sizeof(pix);
}

/*!
 * @brief   内存映射函数
 * @param   filp  : 文件指针
 * @param   vma   : 虚拟内存区域
 * @return  0
 * @note    映射文件时调用, 用于将内核空间帧缓冲映射到用户空间
 * @date    2026/2/18
 */
static int ips_spi_mmap(struct file *filp, struct vm_area_struct *vma)
{
    unsigned long pfn, offset, size;
    unsigned long page_size = PAGE_SIZE;
    // 用户空间请求偏移(字节)
    offset = vma->vm_pgoff << PAGE_SHIFT;
    size = vma->vm_end - vma->vm_start;
    // 检查映射大小与偏移是否合法
    if (size > FB_SIZE || offset + size > FB_SIZE)
    {
        dev_err(&ips_spi->dev, "mmap size/offset invalid: size=%lu, offset=%lu, FB_SIZE=%lu\n",
            size, offset, FB_SIZE);
        return -EINVAL;
    }
    // 检查页对齐
    if (offset % page_size != 0) {
        dev_err(&ips_spi->dev, "offset %lu not page-aligned (page_size=%lu)\n",
                offset, page_size);
        return -EINVAL;
    }
    if (size % page_size != 0) {
        dev_err(&ips_spi->dev, "size %lu not page-aligned (page_size=%lu)\n",
                size, page_size);
        return -EINVAL;
    }

    pfn = virt_to_pfn((unsigned long)((char*)fb_virt + offset));
    if (!pfn)
    {
        dev_err(&ips_spi->dev, "virt_to_pfn failed\n");
        return -EIO;
    }
    if (remap_pfn_range(vma,
                        vma->vm_start,
                        pfn,
                        size,
                        vma->vm_page_prot))
    {
        dev_err(&ips_spi->dev, "remap_pfn_range failed\n");
        return -EIO;
    }
    return 0;
}

/*************************************************************************************************/
/**************************************** 屏幕操作相关函数 ****************************************/
/*************************************************************************************************/
/*!
 * @brief   帧缓冲刷新函数
 * @param   无
 * @return  无
 * @note    将帧缓冲通过 SPI 发送到 IPS 显示屏
 * @date    2026/2/18
 */
void IPSSPI_Flush_Frame(void)
{
    int ret;
    unsigned long flags;
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = (uint8_t*)fb_virt,
        .len = IPSH * IPSW * sizeof(uint16_t),
        .rx_buf = rxbuf,
        .bits_per_word = 8,
    };
    /* 每次整帧刷新前，先设置地址窗口并进入 RAMWR 模式 */
    IPSSPI_Addr_Rst();

    spin_lock_irqsave(&dc_lock, flags);
    my_gpio_set_value(0, &cs_Gpio);
    my_gpio_set_value(1, &dc_Gpio);
    spin_unlock_irqrestore(&dc_lock, flags);

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(ips_spi, &msg);
    if (ret)
        dev_err(&ips_spi->dev, "flush frame failed: %d\n", ret);

    spin_lock_irqsave(&dc_lock, flags);
    my_gpio_set_value(0, &dc_Gpio);
    my_gpio_set_value(1, &cs_Gpio);
    spin_unlock_irqrestore(&dc_lock, flags);
}

/*!
 * @brief    ST7789 初始化
 * @param    type : 0:横屏  1:竖屏
 * @return   无
 * @note     与 TFT 驱动风格保持一致
 * @date     2026/2/18
 */
void IPSSPI_Init(uint8_t type)
{
    my_gpio_set_value(0, &rst_Gpio);
    msleep(120);
    my_gpio_set_value(1, &rst_Gpio);
    msleep(120);

    my_gpio_set_value(0, &cs_Gpio);
    msleep(150);

    /* 软件复位，提升首次上电初始化稳定性 */
    IPSSPI_Write_Cmd(0x01);
    msleep(120);

    IPSSPI_Write_Cmd(0x11); // Sleep Out
    msleep(120);

    IPSSPI_Write_Cmd(0x3A); // 16bit RGB565
    IPSSPI_Write_Byte(0x55);

    IPSSPI_Write_Cmd(0x36); // MADCTL
    if (type)
        IPSSPI_Write_Byte(0x00); // 竖屏
    else
        IPSSPI_Write_Byte(0x60); // 横屏(MX + MV)

    IPSSPI_Write_Cmd(0x3A);
    IPSSPI_Write_Byte(0x05);
    IPSSPI_Write_Cmd(0xB2);
    IPSSPI_Write_Byte(0x0C);
    IPSSPI_Write_Byte(0x0C);
    IPSSPI_Write_Byte(0x00);
    IPSSPI_Write_Byte(0x33);
    IPSSPI_Write_Byte(0x33);
    IPSSPI_Write_Cmd(0xB7);
    IPSSPI_Write_Byte(0x35);
    IPSSPI_Write_Cmd(0xBB);
    IPSSPI_Write_Byte(0x19);
    IPSSPI_Write_Cmd(0xC0);
    IPSSPI_Write_Byte(0x2C);
    IPSSPI_Write_Cmd(0xC2);
    IPSSPI_Write_Byte(0x01);
    IPSSPI_Write_Cmd(0xC3);
    IPSSPI_Write_Byte(0x12);
    IPSSPI_Write_Cmd(0xC4);         
    IPSSPI_Write_Byte(0x20);
    IPSSPI_Write_Cmd(0xC6);
    IPSSPI_Write_Byte(0x0F);
    IPSSPI_Write_Cmd(0xD0);
    IPSSPI_Write_Byte(0xA4);
    IPSSPI_Write_Byte(0xA1);
    IPSSPI_Write_Cmd(0xE0);
    IPSSPI_Write_Byte(0xD0);
    IPSSPI_Write_Byte(0x04);
    IPSSPI_Write_Byte(0x0D);
    IPSSPI_Write_Byte(0x11);
    IPSSPI_Write_Byte(0x13);
    IPSSPI_Write_Byte(0x2B);
    IPSSPI_Write_Byte(0x3F);
    IPSSPI_Write_Byte(0x54);
    IPSSPI_Write_Byte(0x4C);
    IPSSPI_Write_Byte(0x18);
    IPSSPI_Write_Byte(0x0D);
    IPSSPI_Write_Byte(0x0B);
    IPSSPI_Write_Byte(0x1F);
    IPSSPI_Write_Byte(0x23);
    IPSSPI_Write_Cmd(0xE1);
    IPSSPI_Write_Byte(0xD0);
    IPSSPI_Write_Byte(0x04);
    IPSSPI_Write_Byte(0x0C);
    IPSSPI_Write_Byte(0x11);
    IPSSPI_Write_Byte(0x13);
    IPSSPI_Write_Byte(0x2C);
    IPSSPI_Write_Byte(0x3F);
    IPSSPI_Write_Byte(0x44);
    IPSSPI_Write_Byte(0x51);
    IPSSPI_Write_Byte(0x2F);
    IPSSPI_Write_Byte(0x1F);
    IPSSPI_Write_Byte(0x1F);
    IPSSPI_Write_Byte(0x20);
    IPSSPI_Write_Byte(0x23);
    IPSSPI_Write_Cmd(0x21);
    IPSSPI_Write_Cmd(0x29);
    /* Display ON 后给控制器稳定时间，避免首次运行丢首帧 */
    msleep(120);
    IPSSPI_Addr_Rst();
}

/*!
 * @brief    写命令
 * @param    cmd : 命令
 * @return   无
 * @date     2026/2/18
 */
void IPSSPI_Write_Cmd(uint8_t cmd)
{
    int ret;
    unsigned long flags;
    static uint8_t txbuf[1], rxbuf_local[1];
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = txbuf,
        .len = 1,
        .rx_buf = rxbuf_local,
        .bits_per_word = 8,
    };
    spin_lock_irqsave(&dc_lock, flags);
    my_gpio_set_value(0, &cs_Gpio);
    my_gpio_set_value(0, &dc_Gpio);
    txbuf[0] = cmd;
    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(ips_spi, &msg);
    if (ret)
        printk(KERN_ERR "SPI transfer cmd : 0x%x, failed: %d\n", txbuf[0], ret);
    my_gpio_set_value(1, &dc_Gpio);
    my_gpio_set_value(1, &cs_Gpio);
    spin_unlock_irqrestore(&dc_lock, flags);
}

/*!
 * @brief    写字节
 * @param    data : 字节数据
 * @return   无
 * @date     2026/2/18
 */
void IPSSPI_Write_Byte(uint8_t data)
{
    int ret;
    unsigned long flags;
    static uint8_t txbuf[1], rxbuf_local[1];
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = txbuf,
        .len = 1,
        .rx_buf = rxbuf_local,
        .bits_per_word = 8,
    };
    spin_lock_irqsave(&dc_lock, flags);
    my_gpio_set_value(0, &cs_Gpio);
    my_gpio_set_value(1, &dc_Gpio);
    txbuf[0] = data;
    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(ips_spi, &msg);
    if (ret)
        printk(KERN_ERR "SPI transfer data : 0x%x, failed: %d\n", txbuf[0], ret);
    my_gpio_set_value(0, &dc_Gpio);
    my_gpio_set_value(1, &cs_Gpio);
    spin_unlock_irqrestore(&dc_lock, flags);
}

/*!
 * @brief    写字
 * @param    data : 16位数据
 * @return   无
 * @date     2026/2/18
 */
void IPSSPI_Write_Word(uint16_t data)
{
    int ret;
    unsigned long flags;
    static uint8_t txbuf[2], rxbuf_local[2];
    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = txbuf,
        .len = 2,
        .rx_buf = rxbuf_local,
        .bits_per_word = 8,
    };
    spin_lock_irqsave(&dc_lock, flags);
    my_gpio_set_value(0, &cs_Gpio);
    my_gpio_set_value(1, &dc_Gpio);
    txbuf[0] = data >> 8;
    txbuf[1] = data & 0xFF;
    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    ret = spi_sync(ips_spi, &msg);
    if (ret)
        printk(KERN_ERR "SPI transfer word failed: %d\n", ret);
    my_gpio_set_value(0, &dc_Gpio);
    my_gpio_set_value(1, &cs_Gpio);
    spin_unlock_irqrestore(&dc_lock, flags);
}

/*!
 * @brief    重置地址窗口
 * @param    无
 * @return   无
 * @date     2026/2/18
 */
void IPSSPI_Addr_Rst(void)
{
    if (display_status == 0)
    {
        /* 横屏：320 x 240 */
        IPSSPI_Write_Cmd(0x2A);
        IPSSPI_Write_Byte(0x00);
        IPSSPI_Write_Byte(0x00);
        IPSSPI_Write_Byte(0x01);
        IPSSPI_Write_Byte(0x3F);

        IPSSPI_Write_Cmd(0x2B);
        IPSSPI_Write_Byte(0x00);
        IPSSPI_Write_Byte(0x00);
        IPSSPI_Write_Byte(0x00);
        IPSSPI_Write_Byte(0xEF);
    }
    else
    {
        /* 竖屏：240 x 320 */
        IPSSPI_Write_Cmd(0x2A);
        IPSSPI_Write_Byte(0x00);
        IPSSPI_Write_Byte(0x00);
        IPSSPI_Write_Byte(0x00);
        IPSSPI_Write_Byte(0xEF);

        IPSSPI_Write_Cmd(0x2B);
        IPSSPI_Write_Byte(0x00);
        IPSSPI_Write_Byte(0x00);
        IPSSPI_Write_Byte(0x01);
        IPSSPI_Write_Byte(0x3F);
    }

    IPSSPI_Write_Cmd(0x2C);
}

/*************************************************************************************************/
/**************************************** GPIO相关函数 ********************************************/
/*************************************************************************************************/
/*!
 * @brief   初始化 GPIO
 * @param   gpio : GPIO 编号
 * @param   Gpio : GPIO 描述结构
 * @return  失败返回错误码，成功返回 0
 */
int my_gpio_init(uint8_t gpio, struct HardwareGPIO *Gpio)
{
    // 获取引脚编号
    Gpio->gpio = gpio;
    // 获取 GPIO 寄存器映射地址
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
    // 将对应引脚配置为 GPIO 模式
    writel(readl(Gpio->gpio_reuse) & ~(0b11 << (gpio % 16 * 2)), Gpio->gpio_reuse);
    return 0;
}

/*!
 * @brief   设置 GPIO 模式
 * @param   mode : GPIO_Mode_In 输入模式，GPIO_Mode_Out 输出模式
 * @param   Gpio : GPIO 描述结构
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
 * @param   val  : GPIO 电平值
 * @param   Gpio : GPIO 描述结构
 * @return  失败返回 -1，成功返回设置值
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
 * @brief   读取 GPIO 高低电平
 * @param   Gpio : GPIO 描述结构
 * @return  返回读取到的电平值
 */
int my_gpio_get_value(struct HardwareGPIO *Gpio)
{
    return ((readl(Gpio->gpio_base + LS_GPIO_I_OFFSET(Gpio->gpio % 8)) & BIT(Gpio->gpio % 8)) == BIT(Gpio->gpio % 8));
}

/*!
 * @brief   释放 GPIO
 * @param   GPIO : GPIO 描述结构
 * @return  无
 */
void my_gpio_release(struct HardwareGPIO *Gpio)
{
    iounmap(Gpio->gpio_base);
    iounmap(Gpio->gpio_reuse);
}

// 注册模块初始化和卸载函数
module_init(spi_driver_init);
module_exit(spi_driver_exit);

MODULE_AUTHOR("LQ_012");
MODULE_DESCRIPTION("ST7789 IPS-SPI Display Driver Terminal");
MODULE_LICENSE("GPL");
