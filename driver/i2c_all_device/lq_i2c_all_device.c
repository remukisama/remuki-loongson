#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/errno.h>

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

/********************************************************************************
 * @brief   加载函数
 ********************************************************************************/
static int __init dev_init(void)
{
    struct i2c_adapter *adapter = NULL;
    struct i2c_board_info i2c_board_info;
    int i = 0;
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
        unsigned short addr_list[2];    // 单个设备的地址列表
        // 清空设备信息结构体
        memset(&i2c_board_info, 0, sizeof(struct i2c_board_info));
        // 设置当前设备名（匹配对应驱动端的name）
        strlcpy(i2c_board_info.type, I2C_DEVS[i].name, I2C_NAME_SIZE);
        // 构建单个设备的地址列表
        addr_list[0] = I2C_DEVS[i].addr;
        addr_list[1] = I2C_CLIENT_END;
        // 探测并注册该设备名对应的I2C设备（遍历所有地址）
        clients[client_count] = i2c_new_probed_device(
            adapter,            // I2C总线适配器
            &i2c_board_info,    // 设备信息
            addr_list,          // 要探测的设备地址列表
            NULL                // 自定义匹配函数(NULL表示默认规则)
        );
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
