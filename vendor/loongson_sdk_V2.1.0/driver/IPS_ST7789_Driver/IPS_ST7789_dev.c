#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/spi/spi.h>

// 设备名称
#define DEV_ID_NAME "loongson,ips-st7789-driver"

// 设备信息（与 TFT18 使用同一组 SPI 引脚）
static struct spi_board_info spi_dev_info = {
    .modalias = DEV_ID_NAME,
    .max_speed_hz = 50000000,   // 最大时钟频率
    .bus_num = 1,               // SPI 总线编号
    .chip_select = 2,           // 片选信号
};

/*!
 * @brief   SPI 设备初始化函数
 * @param   无
 * @return  成功返回 0，失败返回错误码
 * @note    该函数用于初始化 SPI 设备，并注册到系统中
 * @date    2026/2/18
 */
static int __init spi_device_init(void)
{
    struct spi_master *master;
    struct spi_device *spi;
    // 获取 SPI 主控制器
    master = spi_busnum_to_master(spi_dev_info.bus_num);
    if (!master)
    {
        printk(KERN_ERR "Failed to get SPI master on bus %d\n", spi_dev_info.bus_num);
        return -ENODEV;
    }
    // 创建 SPI 设备
    spi = spi_new_device(master, &spi_dev_info);
    if (!spi)
    {
        printk(KERN_ERR "Failed to create SPI device on bus %d, chip select %d\n", spi_dev_info.bus_num, spi_dev_info.chip_select);
        spi_master_put(master);
        return -ENODEV;
    }
    // 释放 SPI 主控制器
    spi_master_put(master);
    printk(KERN_INFO "SPI device %s registered on bus %d, chip select %d\n", spi_dev_info.modalias, spi_dev_info.bus_num, spi_dev_info.chip_select);
    return 0;
}

/*!
 * @brief   找到 SPI 设备回调函数
 * @param   dev  : 设备
 * @param   data : 传递给回调函数的数据
 * @return  0:继续遍历    -1:停止遍历
 * @note    该函数用于 device_for_each_child() 遍历 spi_controller 下的所有子设备，找到指定片选信号的 SPI 设备
 * @date    2026/2/18
 */
static int find_spi_device_callback(struct device *dev, void *data)
{
    struct spi_device *spi = to_spi_device(dev);
    int *chip_select = (int*)data;  // 从 data 中获取片选信号

    if (spi->chip_select == *chip_select)
    {
        return -1;  // 非零值停止遍历
    }
    return 0;
}

/*!
 * @brief   找到 SPI 设备
 * @param   controller  : SPI 主控制器
 * @param   chip_select : 片选信号
 * @return  找到的 SPI 设备
 * @note    该函数通过遍历 spi_controller 下的所有子设备，找到指定片选信号的 SPI 设备
 * @date    2026/2/18
 */
struct spi_device *find_spi_device(struct spi_controller *controller, int chip_select)
{
    int ret = 0;
    struct spi_device *spi = NULL;

    if (!controller)
        return NULL;

    // 遍历 spi_controller 下的所有子设备
    ret = device_for_each_child(&controller->dev, &chip_select, find_spi_device_callback);
    if (ret == -1)
    {
        // 找到设备
        spi = to_spi_device(device_find_child(&controller->dev, &chip_select, find_spi_device_callback));
    }

    return spi;
}

/*!
 * @brief   SPI 设备退出函数
 * @param   无
 * @return  无
 * @note    该函数用于注销 SPI 设备，并释放 SPI 主控制器
 * @date    2026/2/18
 */
static void __exit spi_device_exit(void)
{
    struct spi_master *master;
    struct spi_device *spi;
    // 获取 SPI 主控制器
    master = spi_busnum_to_master(spi_dev_info.bus_num);
    if (!master)
    {
        printk(KERN_ERR "Failed to get SPI master on bus %d\n", spi_dev_info.bus_num);
        return;
    }
    // 找到 SPI 设备
    spi = find_spi_device(master, spi_dev_info.chip_select);
    if (spi)
    {
        spi_unregister_device(spi);
        printk(KERN_INFO "SPI device %s unregistered on bus %d, chip select %d\n", spi_dev_info.modalias, spi_dev_info.bus_num, spi_dev_info.chip_select);
    }
    // 释放 SPI 主控制器
    spi_master_put(master);
    printk(KERN_INFO "SPI device unregistered\n");
}

module_init(spi_device_init);
module_exit(spi_device_exit);

MODULE_AUTHOR("LQ_012");
MODULE_DESCRIPTION("ST7789 IPS device end");
MODULE_LICENSE("GPL");
