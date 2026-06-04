#include "lq_all_demo.hpp"
#include "user_app.hpp"
#include <iostream>
#include <unistd.h>

/**
 * @brief    此 Demo 专门用于测试底盘编码器的连线与极性。
 * @details  运行此 demo 后：
 *           1. 程序会自动给左右电机微弱的占空比使其慢速正转。
 *           2. 终端会每隔 100ms 打印一次左右编码器的读数。
 *           3. 你可以借此确认：电机转起时，右侧编码器是否有数值？数值是否为正？
 */
void lq_car_encoder_test_demo(void)
{
    printf("\n======================================================\n");
    printf("   欢迎使用三轮车底盘【左/右编码器】裸机测试程序\n");
    printf("======================================================\n\n");

    // 1. 初始化电机引脚
    motor_Init();
    printf("-> [1/3] 电机外设初始化完成。\n");

    // 2. 为了安全，先清零一遍计数器
    float trash_l = encoder_num_L();
    float trash_r = encoder_num_R();
    (void)trash_l; 
    (void)trash_r;
    printf("-> [2/3] 编码器历史数据已清零。\n");

    printf("-> [3/3] 即将给左右轮施加微弱动力正转，并开始打印测速数据...\n\n");
    usleep(1000 * 1000); // 停顿 1 秒给用户准备时间

    // 测试占空比（最大 10000，这里给 2500 提供足够低速扭矩）
    int test_duty = 2500;  
    
    // 给电机恒定动力（直接调用你的开环函数）
    motor_Duty_L(test_duty);
    motor_Duty_R(test_duty);

    int count = 0;
    while (!exit_flag)
    {
        // 每 100 毫秒读取一次编码器并清零（模拟闭环中的采集频率）
        float enc_l = encoder_num_L();
        float enc_r = encoder_num_R();

        printf("循环第 [%4d] 次 | 电机占空比: %d | 左轮读数: %7.2f | 右轮读数: %7.2f\n", ++count, test_duty, enc_l, enc_r);

        // 如果想测试手动拨动轮子，也可以把上面的 motor_Duty 注释掉，或者改为 0。
        // 比如如果转得太快，可以在循环第 30 次后停止电机，只看空转时的编码器数据：
        if (count == 30) {
            printf("\n--- 【注意】测试已运行 3 秒，为安全起见切断电机动力，但会继续读编码器！ ---\n");
            printf("--- （现在你可以用手去拨动右轮子，看看数值跟不跟着变） ---\n\n");
            motor_Duty_L(0);
            motor_Duty_R(0);
        }

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 延时 100ms
        select(0, NULL, NULL, NULL, &tv); 
    }

    // 退出时关停电机
    motor_Duty_L(0);
    motor_Duty_R(0);
    printf("-> 编码器测试结束。\n");
}
