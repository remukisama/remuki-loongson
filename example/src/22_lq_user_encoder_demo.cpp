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
        int enc_l = encoder_num_L();
        int enc_r = encoder_num_R();

        printf("Encoder Left: %6d  |  Encoder Right: %6d\n", enc_l, enc_r);
        
        // 延时 20ms，与你闭环控制的周期类似
        usleep(20000); 
    }
}
