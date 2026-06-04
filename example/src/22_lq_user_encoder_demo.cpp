#include "lq_all_demo.hpp"
#include "user_app.hpp"
#include <iostream>
#include <unistd.h>

void lq_user_encoder_demo(void)
{
    std::cout << "========= lq_user_encoder_demo start =========" << std::endl;
    
    // 如果电机需要初始化，可以调用 motor_Init();
    // motor_Init();

    while (1)
    {
        float enc_l = encoder_num_L();
        float enc_r = encoder_num_R();

        printf("Encoder Left: %7.2f  |  Encoder Right: %7.2f\n", enc_l, enc_r);
        
        // 延时 20ms，与你闭环控制的周期类似
        usleep(20000); 
    }
}
