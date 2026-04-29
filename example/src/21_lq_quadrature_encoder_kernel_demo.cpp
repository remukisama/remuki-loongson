#include <iostream>
#include <unistd.h>
#include "lq_app_inc.hpp"
#include "lq_reg_quadrature_encoder.hpp"

// -------------------------------------------------------------
// 【重要提示】
// 运行本程序前，请确保您已经加载了内核模块 lq_encoder_irq.ko ！
// 执行命令示例（假设左轮接88/89，右轮接67/75）：
// cd driver/encoder_irq_driver
// make
// insmod lq_encoder_irq.ko pinA=88,67 pinB=89,75
// 之后方可运行本二进制文件！
// -------------------------------------------------------------

void lq_quadrature_encoder_kernel_demo(void)
{
    std::cout << "========== Linux Kernel Based 2-Channel Quadrature Encoder Demo ==========" << std::endl;

    // 参数表示设备号：0对应 /dev/lq_encoder0(左轮), 1对应 /dev/lq_encoder1(右轮)
    ls_quadrature_encoder encoder_left(0);
    ls_quadrature_encoder encoder_right(1);

    // 清零当前数值
    encoder_left.encoder_reset_counter();
    encoder_right.encoder_reset_counter();

    while (1) {
        float enc_l_val = encoder_left.encoder_get_count();
        int8_t dir_l = encoder_left.encoder_get_direction();

        float enc_r_val = encoder_right.encoder_get_count();
        int8_t dir_r = encoder_right.encoder_get_direction();

        std::cout << "Left Motor [Count: " << enc_l_val << ", Dir: " << (int)dir_l 
                  << "] | Right Motor [Count: " << enc_r_val << ", Dir: " << (int)dir_r << "]" << std::endl;
        
        usleep(100000); // 间隔100ms打印一次
    }
}
