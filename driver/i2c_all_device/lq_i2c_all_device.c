#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/err.h>

#define I2C_PROBE_RETRIES          5
#define I2C_PROBE_RETRY_DELAY_MS   100

// 定义设备信息结构体
struct i2c_dev_info {
    const char    *name;    // 设备名称
    unsigned short addr;    // 设备地址
};

// 定义设备列表 - 每个设备对应一个地址
static const struct i2c_dev_info I2C_DEVS[] = {
    {"ls,lq_i2c_mpu6050" , 0x68},   // MPU6050
    {"ls,lq_i2c_lsm6dsr" , 0x6b},   // LSM6DSR
    {"ls,lq_i2c_vl53l0x" , 0x29},   // VL53L0X
    {"ls,lq_i2c_icm42688", 0x69},   // ICM42688
    { NULL, 0 } // 结束标志
};

// 保存多个注册成功的I2C client
static struct i2c_client *clients[8] = {NULL};
static int client_count = 0;

static struct i2c_client *lq_i2c_probe_device_with_retry(
    struct i2c_adapter *adapter,
    const struct i2c_dev_info *dev_info)
{
    struct i2c_board_info i2c_board_info;
    struct i2c_client *client = NULL;
    unsigned short addr_list[2];
    int retry;

    addr_list[0] = dev_info->addr;
    addr_list[1] = I2C_CLIENT_END;

    for (retry = 0; retry < I2C_PROBE_RETRIES; retry++) {
        memset(&i2c_board_info, 0, sizeof(i2c_board_info));
        strlcpy(i2c_board_info.type, dev_info->name, I2C_NAME_SIZE);

        client = i2c_new_probed_device(adapter, &i2c_board_info, addr_list, NULL);
        if (client && !IS_ERR(client))
            return client;

        if (IS_ERR(client)) {
            printk(KERN_WARNING
                   "LQ_I2C_ALL_DEV: probe device %s addr=0x%02x failed, ret=%ld, retry=%d/%d\n",
                   dev_info->name, dev_info->addr, PTR_ERR(client),
                   retry + 1, I2C_PROBE_RETRIES);
            client = NULL;
        } else {
            printk(KERN_WARNING
                   "LQ_I2C_ALL_DEV: probe device %s addr=0x%02x failed, retry=%d/%d\n",
                   dev_info->name, dev_info->addr,
                   retry + 1, I2C_PROBE_RETRIES);
        }

        if (retry + 1 < I2C_PROBE_RETRIES)
            msleep(I2C_PROBE_RETRY_DELAY_MS);
    }

    return NULL;
}

/********************************************************************************
 * @brief   加载函数
 ********************************************************************************/
static int __init dev_init(void)
{
    struct i2c_adapter *adapter = NULL;
    int i = 0;
    client_count = 0;
    // 获取I2C总线适配器（当前使用第1个I2C总线适配器）
    adapter = i2c_get_adapter(1);
    if (!adapter)
    {
        printk("LQ_I2C_ALL_DEV: get i2c adapter 1 failed!\n");
        return -ENODEV;
    }
    printk("LQ_I2C_ALL_DEV init function, start probe multi devices\n");
    // 遍历所有设备名称
    while (I2C_DEVS[i].name != NULL && client_count < ARRAY_SIZE(clients)) {
        // 探测并注册该设备名对应的I2C设备。部分板卡上I2C刚释放后首次探测会短暂失败，需重试。
        clients[client_count] = lq_i2c_probe_device_with_retry(adapter, &I2C_DEVS[i]);
        // 检查注册结果
        if (clients[client_count]) {
            printk(KERN_INFO "LQ_I2C_ALL_DEV: register device %s success, addr=0x%02x!\n", I2C_DEVS[i].name, clients[client_count]->addr);
            client_count++;
        } else {
            printk(KERN_WARNING "LQ_I2C_ALL_DEV: probe device %s failed!\n", I2C_DEVS[i].name);
        }
        i++;
    }
    // 释放适配器
    i2c_put_adapter(adapter);
    // 如果至少注册成功一个设备，返回0，否则返回错误
    if (client_count > 0)
        return 0;
    else
        return -ENODEV;
}

/********************************************************************************
 * @brief   卸载函数
 ********************************************************************************/
static void __exit dev_exit(void)
{
    int i;
    // 遍历所有注册的client，逐个注销
    for (i = 0; i < client_count; i++) {
        printk(KERN_INFO "LQ_I2C_ALL_DEV: unregister device %s, addr=0x%02x!\n", clients[i]->name, clients[i]->addr);
        i2c_unregister_device(clients[i]);
        clients[i] = NULL;  // 清空指针，避免野指针
    }
    // 重置注册计数
    client_count = 0;
    printk("LQ_I2C_ALL_DEV exit function, all devices unregistered\n");
}

module_init(dev_init);                              // 指定加载函数
module_exit(dev_exit);                              // 指定卸载函数
MODULE_AUTHOR("LQ_012 <chiusir@163.com>");          // 作者以及邮箱
MODULE_DESCRIPTION("支持多驱动匹配的I2C1设备端模块");  // 模块简单介绍
MODULE_VERSION("2.1");                              // 版本号
MODULE_LICENSE("GPL");                              // 许可证声明
