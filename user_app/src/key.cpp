#include "user_app.hpp"
#include <atomic>
#include <iostream>


ls_gpio KO_O(PIN_44, GPIO_MODE_IN);
ls_gpio KO_1(PIN_45, GPIO_MODE_IN);

/******************************/
char KO_O_flag,KO_1_flag;
char KO_O_ZT,KO_1_ZT;
char KO_O_ZT_last,KO_1_ZT_last;
/******************************/

char image_show_flag=1;
char car_start=0;

/************************************
图像显示：：   1 显示在TFT屏幕上  
            2 显示在中端上
            3 显示舵机PID，显示舵机输出
************************************/
void image_show(void)
{
//    图像显示***********************************************
                for(int i=0;i<60;i++)
                {
                    for(int j=0;j<80;j++)
                    {
                        // if(i==22)
                        // {
                        //     printf("7");
                        // }
                        // else
                            printf("%d",img3[i][j]);
                    }
                    printf("\n");
                }
                printf("\n\n\n");


                // printf("%.2f\n",stree_erro);
                // printf("out:%d\n",stree_out);

                // printf("赛道偏差:%.2f\n", ImageStatus.Det_True - ImageStatus.MiddleLine);
                // printf("左轮目标:%d, 实际:%d\n", Speed_Goal_l, speed_encoder_L);
                // printf("右轮目标:%d, 实际:%d\n", Speed_Goal_r, speed_encoder_R);

}

void Key_Get(void)
{
    KO_O_ZT_last=KO_O_ZT;
    KO_1_ZT_last=KO_1_ZT;

    KO_O_ZT=KO_O.gpio_level_get();
    KO_1_ZT=KO_1.gpio_level_get();

    
    if(KO_O_ZT && KO_O_ZT_last==0)  KO_O_flag=1;
    if(KO_1_ZT && KO_1_ZT_last==0)  KO_1_flag=1;

    Key_KZ();

    if(car_start==2)
    {
        sleep(1);
        // 启动时双轮速度一致，直线起步
        Speed_Goal_r=50;
        Speed_Goal_l=50;
        
    }
    else if(car_start==3)
    {
        Speed_Goal_r=0;
        Speed_Goal_l=0;
    }

}

void Key_KZ(void)
{
    if(KO_O_flag)
    {
        // printf("KOKO:%d\n",car_start);
        car_start++;
        KO_O_flag=0;
    }

    if(KO_1_flag)
    {
        // TFTSPI_CLS(u16WHITE); 
        printf("K1K1\n");
        KO_1_flag=0;
        
        
    }
}

