#include "lq_all_demo.hpp"
#include <unistd.h>
#include <iostream>

/********************************************************************************
 * @brief   正交编码器测试例程
 * @param   none.
 * @return  none.
 * @note    此例程展示了如何使用 lq_reg_quadrature_encoder 读取正交编码器的数值
 ********************************************************************************/
// void lq_quadrature_encoder_demo(void)
// {
//     // 测试引脚设置: A相->PIN_64, B相->PIN_65
//     // 请根据实际接线情况修改引脚
//     gpio_pin_t pinA = PIN_67;
//     gpio_pin_t pinB = PIN_75;

//     std::cout << "Starting Quadrature Encoder Demo..." << std::endl;
//     std::cout << "Phase A Pin: " << pinA << ", Phase B Pin: " << pinB << std::endl;

//     // 实例化正交编码器对象
//     ls_quadrature_encoder my_encoder(pinA, pinB);

//     ls_atim_pwm pwm1(ATIM_PWM0_PIN81, 10000, 3000, ATIM_PWM_POL_INV);
//     ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 10000, 3000, ATIM_PWM_POL_INV);
//     ls_gpio pwm1dir(PIN_21, GPIO_MODE_OUT);
//     ls_gpio pwm2dir(PIN_22, GPIO_MODE_OUT);
//     pwm1.atim_pwm_enable();
//     pwm2.atim_pwm_enable();

//     // 重置计数器
//     my_encoder.encoder_reset_counter();

//     while (1) 
//     {
//         pwm1dir.gpio_level_set(GPIO_HIGH);
//         pwm2dir.gpio_level_set(GPIO_HIGH);
//         // 获取编码器当前计数值
//         float count = my_encoder.encoder_get_count();
//         // 获取编码器方向 (-1 或 1)
//         int8_t dir = my_encoder.encoder_get_direction();
        
//         std::string dir_str = (dir == 1) ? "Forward" : (dir == -1) ? "Reverse" : "Unknown";

//         // 打印输出
//         std::cout << "Quadrature Encoder Count: " << count 
//                   << " | Direction: " << dir_str << std::endl;

//         // 延时 500ms
//         usleep(500000);
//     }
// }
