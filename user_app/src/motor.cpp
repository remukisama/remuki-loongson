// #include "user_app.hpp"
// #include <atomic>
// #include <iostream>

// #define MAX_PWM_DUTY 2000 // 最大PWM限幅

// // 在文件开头、函数定义之前声明
// std::atomic<bool> exit_flag(false);  // 原子变量，多线程安全
// /* 电机PWM */
// ls_atim_pwm motor_R(ATIM_PWM0_PIN81, 10000, 5000, ATIM_PWM_POL_NORMAL);
// ls_atim_pwm motor_L(ATIM_PWM1_PIN82, 10000, 5000, ATIM_PWM_POL_NORMAL);
// /* 电机方向GPIO */
// //方向引脚输出0是正转，1是反转
// ls_gpio motor_R_Dir(PIN_21, GPIO_MODE_OUT);
// ls_gpio motor_L_Dir(PIN_22, GPIO_MODE_OUT);

// float Left_Speed_Co = 0.0010, Right_Speed_Co =0.0010;
// int Speed_Goal=0;
// float CS=0.1;//0.55 0.57  0.58

// int ZD_Speed_Goal;
// int WD_Speed_Goal;

// // ========== 三轮车差速转向专用参数 ==========
// float STEER_KP = 2.5f;        // 转向比例系数：减小P，防止转向过猛导致内切
// float STEER_KD = 40.0f;       // 转向微分阻尼系数（D项）：保持阻尼
// float MAX_DIFF_RATIO = 1.2f;  // 最大差速比例：减小最大差速，防止极其锐利的转弯
// int DEAD_BAND = 3;            // 偏差死区：防直道抖动
// int MIN_SPEED = 6;            // 【解决内切关键】禁止内侧轮倒转，保持保底向前速度，拉大转弯半径
// // ============================================

// //记录编码器
// int Record_encoder=0;

// void motor_Init(void)
// {
//     // 设置占空比
//     motor_R.atim_pwm_set_active_low_duty(0);
//     motor_L.atim_pwm_set_active_low_duty(0);
//     // 使能
//     printf("R:OK!");
//     motor_R.atim_pwm_enable();
//     motor_L.atim_pwm_enable();
//     motor_R_Dir.gpio_level_set(GPIO_LOW);
//     motor_L_Dir.gpio_level_set(GPIO_LOW);
// }


// void motor_Duty_R(int duty)
// {
//     // 【调试排查】强制打印最后分发给右轮的占空比和方向
//     // 观察这个输出，看系统究竟是想让它前进还是倒退
//     // printf("R_duty: %d\n", duty);
    
//     if (duty <= 0)
//     {
//         motor_R.atim_pwm_set_active_low_duty(-duty);
//         motor_R_Dir.gpio_level_set(GPIO_HIGH); // 反转：观察电机是否反转
//     }
//     else
//     {
//         motor_R.atim_pwm_set_active_low_duty(duty);
//         motor_R_Dir.gpio_level_set(GPIO_LOW);  // 正转：观察电机是否正转
//     }

// }


// void motor_Duty_L(int duty)
// {
//     // printf("L_duty: %d\n", duty);

//     if (duty <= 0)
//     {
//         motor_L.atim_pwm_set_active_low_duty(-duty);
//         motor_L_Dir.gpio_level_set(GPIO_HIGH); 
//     }
//     else
//     {
//         motor_L.atim_pwm_set_active_low_duty(duty);
//         motor_L_Dir.gpio_level_set(GPIO_LOW);
//     }
// }



// //左闭环变量
// int Speed_Goal_l=0;
// int speed_I_l, speed_P_l;
// int speed_out_l;
// int speed_erro_L, speed_last_erro_L;
// int speed_encoder_L;

// //右闭环变量
// int Speed_Goal_r=0;
// int speed_I_r, speed_P_r;
// int speed_out_r;
// int speed_erro_R, speed_last_erro_R;
// int speed_encoder_R;

// // //闭环参数初始化
// // void Speed_decision(void)
// // {
    
// //     speed_I_l = 65;
// //     speed_P_l = 140;

// //     speed_I_r = 65;
// //     speed_P_r = 140;


// //     WD_Speed_Goal=79;   //弯道速8
// //     ZD_Speed_Goal=86;     //直xian88
// //     // car_start=0;
// // }

// void Speed_decision(void)
// {
//     // 三轮车初始适配参数 (已针对 512线四倍频 编码器放大4倍进行适配调整)
//     speed_I_l = 9;   // 原35，因为编码器计数值扩大了4倍，P值缩小4倍以保持同等控制力度
//     speed_P_l = 22;  // 原90，缩小4倍
//     speed_I_r = 9;
//     speed_P_r = 22;
//     WD_Speed_Goal=14;  // 原30，目标速度值扩大4倍
//     ZD_Speed_Goal=20;  // 原70，目标速度值扩大4倍
//     // car_start=0;
// }

// void Speed_PID_L(void)
// {
//     speed_erro_L = Speed_Goal_l - speed_encoder_L;
    
//     // 【修复致命错误】标准增量式PID算法中，P项必须是“加”号！
//     // 否则误差减小时反而会加速电机，引发正反馈震荡甚至反转倒车。
//     speed_out_l += speed_I_l * speed_erro_L + speed_P_l * (speed_erro_L - speed_last_erro_L);
    
//     speed_last_erro_L = speed_erro_L;

//     // 限幅保护
//     if (speed_out_l > MAX_PWM_DUTY) speed_out_l = MAX_PWM_DUTY;
//     if (speed_out_l < -MAX_PWM_DUTY) speed_out_l = -MAX_PWM_DUTY;

//     // 停车清零
//     if (Speed_Goal_l == 0) {
//         speed_out_l = 0;
//         speed_last_erro_L = 0;
//     }
// }


// void Speed_PID_R(void)
// {
//     speed_erro_R = Speed_Goal_r - speed_encoder_R;
    
//     // 【修复致命错误】标准增量式PID算法中，P项必须是“加”号！
//     speed_out_r += speed_I_r * speed_erro_R + speed_P_r * (speed_erro_R - speed_last_erro_R);
    
//     speed_last_erro_R = speed_erro_R;

//     // 限幅保护
//     if (speed_out_r > MAX_PWM_DUTY) speed_out_r = MAX_PWM_DUTY;
//     if (speed_out_r < -MAX_PWM_DUTY) speed_out_r = -MAX_PWM_DUTY;

//     // 停车清零
//     if (Speed_Goal_r == 0) {
//         speed_out_r = 0;
//         speed_last_erro_R = 0;
        
//     }
// }


// //差速
// uint16_t CS_LM=50;
// // void speed_diff()
// // {
// //     float y,x;

// //     // if(ImageFlag.image_element_rings_flag != 0 && ImageFlag.image_element_rings == 1) {CS=ZYCS;}//左圆环
// //     // else if(ImageFlag.image_element_rings_flag != 0 && ImageFlag.image_element_rings == 2) {CS=YYCS;}//右圆环
// //     // else{CS=WDCS;}//

// //     if(stree_out >= stree_mid)//左转//240
// //     {
// //         x = (stree_out - stree_mid) * Left_Speed_Co;
// //         if(x > CS_LM)//240
// //         {
// //             x = CS_LM;
// //         }
// //         y = -0.0145344 + 0.000785637*x - 0.00014484*x*x;
// //         Speed_Goal_r = Speed_Goal *(1 + CS * y * 0.2);
// //         Speed_Goal_l = Speed_Goal * (1 - CS * y);//0.4100
// //     }
// //     else if(stree_out < stree_mid)//右转
// //     {
// //         x = (stree_mid - stree_out) * Right_Speed_Co;
// //         if(x>CS_LM)
// //         {
// //             x = CS_LM;
// //         }
// //         y = -0.0145344 + 0.000785637*x - 0.00014484*x*x;
// //         Speed_Goal_l = Speed_Goal  *(1 + CS * y * 0.2);
// //         Speed_Goal_r = Speed_Goal * (1 - CS * y);//0.3000
// //     }
// // }

// void speed_diff()
// {
//     // 1. 获取赛道中心偏差（核心转向输入）
//     // 已将颠倒图像翻转正向（车头向左/右，图像黑线就偏左/右）
//     // 偏差值：>0 赛道中心偏右，需要向右转，令左轮加速，右轮减速
//     //         <0 赛道中心偏左，需要向左转，令左轮减速，右轮加速
//     // 发现小车转弯打反了，所以这里把 Det_True 和 MiddleLine 颠倒一下减法即可
//     float raw_steer_error = ImageStatus.MiddleLine - ImageStatus.Det_True;

//     // 【新增】：一阶低通滤波，防止视觉丢线导致的误差瞬间反转
//     static float steer_error = 0;
//     // 滤波系数 alpha (0~1)：值越小抗干扰越强，但转向越延迟。如果还是跳变可以改小到0.4
//     float alpha = 0.6f; 
//     steer_error = steer_error * (1.0f - alpha) + raw_steer_error * alpha;

//     // 【开机保护】防止摄像头还没出数据时（Det_True默认为0）产生极大的 steer_error 导致车原地打转
//     static bool camera_is_ready = false;
//     if (!camera_is_ready) {
//         if (ImageStatus.Det_True == 0) {
//             Speed_Goal_l = 0; // 直接让电机目标速度为0，强制停车等待摄像头就绪
//             Speed_Goal_r = 0;
//             return;
//         } else {
//             camera_is_ready = true; // 捕捉到第一次有效解算数据，解除保护
//         }
//     }
    
//     // 用于PD控制的静态变量，记录上次误差
//     static float last_steer_error = 0;
//     // 计算偏差变化率（微分D项预测趋势）
//     float diff_steer = steer_error - last_steer_error;
    
//     // 弯道偏差阈值（根据你的误差范围调整，比如±50）
//     if (abs(steer_error) > 50) {
//         Speed_Goal = WD_Speed_Goal; // 弯道强制降速
//     }

//     // 2. 死区处理，直道小偏差不做转向，防止抖动
//     if (abs(steer_error) < DEAD_BAND)
//     {
//         Speed_Goal_l = Speed_Goal;
//         Speed_Goal_r = Speed_Goal;
//         // 【关键】死区内即便不转向，也必须更新上次误差，以免出死区瞬间 D 项计算出巨大突变
//         last_steer_error = steer_error;
//         return;
//     }

//     // 3. 计算差速修正量，升级为 PD 控制！（比例+微分）
//     // P 决定转弯力道，D 提供阻尼抵抗惯性导致的摇摆(画龙)
//     float diff_speed = steer_error * STEER_KP + diff_steer * STEER_KD;
    
//     // 闭环迭代：保存当前误差到下一次用
//     last_steer_error = steer_error;
    
//     // 4. 差速限幅，防止速差过大导致甩尾/原地打转
//     float max_diff = Speed_Goal * MAX_DIFF_RATIO;
//     if (diff_speed > max_diff)  diff_speed = max_diff;
//     if (diff_speed < -max_diff) diff_speed = -max_diff;

//     // 5. 分配左右轮目标速度（核心转向逻辑）
//     // 注意修正方向：
//     // 若 steer_error > 0 (diff_speed > 0)，表示赛道偏右，需要车向右打方向 -> 左轮加速(+)，右轮减速(-)
//     // 若 steer_error < 0 (diff_speed < 0)，表示赛道偏左，需要车向左打方向 -> 左轮减速(-)，右轮加速(+)
//     Speed_Goal_l = Speed_Goal + diff_speed;
//     Speed_Goal_r = Speed_Goal - diff_speed;

//     // 限制单边车轮转速不能反转
//     if (Speed_Goal_l < MIN_SPEED) Speed_Goal_l = MIN_SPEED;
//     if (Speed_Goal_r < MIN_SPEED) Speed_Goal_r = MIN_SPEED;

//     // 可选：最高转速限幅，防止超电机额定转速
//     if (Speed_Goal_l > MAX_SPEED) Speed_Goal_l = MAX_SPEED;
//     if (Speed_Goal_r > MAX_SPEED) Speed_Goal_r = MAX_SPEED;
// }

// // void speed_diff() {
// //     // 1. 获取图像偏差+陀螺仪修正后的总偏差
// //     float steer_error = ImageStatus.Det_True - ImageStatus.MiddleLine;
    
// //     // 2. 陀螺仪Yaw角速度修正（抑制瞬时抖动）
// //     // 原理：转向角速度与偏差方向一致时，放大偏差；相反时，减小偏差
// //     if (abs(steer_error) > DEAD_BAND) {
// //         if (steer_error > 0) { // 需左转，Yaw角速度为正（gz>0）
// //             steer_error += icm_gz * 0.3f; // 系数0.3可调试
// //         } else { // 需右转，Yaw角速度为负（gz<0）
// //             steer_error += icm_gz * 0.3f;
// //         }
// //     }
    
// //     // 3. 死区处理（原逻辑保留）
// //     if (abs(steer_error) < DEAD_BAND) {
// //         Speed_Goal_l = Speed_Goal;
// //         Speed_Goal_r = Speed_Goal;
// //         return;
// //     }
    
// //     // 4. 圆环场景优化：通过Yaw角速度限制最大转向幅度
// //     float max_diff = Speed_Goal * MAX_DIFF_RATIO;
// //     if (ImageStatus.Road_type == LeftCirque || ImageStatus.Road_type == RightCirque) {
// //         max_diff *= 0.8f; // 圆环内限幅缩小20%，避免甩尾
// //         // 若Yaw角速度过大，进一步减小速差
// //         if (abs(icm_gz) > 50) { // 50°/s为阈值，可调试
// //             max_diff *= 0.6f;
// //         }
// //     }
    
// //     // 5. 后续逻辑保留（原差速计算、限幅）
// //     float diff_speed = steer_error * STEER_KP;
// //     if (diff_speed > max_diff) diff_speed = max_diff;
// //     if (diff_speed < -max_diff) diff_speed = -max_diff;
    
// //     Speed_Goal_l = Speed_Goal - diff_speed;
// //     Speed_Goal_r = Speed_Goal + diff_speed;
    
// //     // 单轮最低转速保护
// //     if (Speed_Goal_l < MIN_SPEED) Speed_Goal_l = MIN_SPEED;
// //     if (Speed_Goal_r < MIN_SPEED) Speed_Goal_r = MIN_SPEED;
// //     if (Speed_Goal_l > MAX_SPEED) Speed_Goal_l = MAX_SPEED;
// //     if (Speed_Goal_r > MAX_SPEED) Speed_Goal_r = MAX_SPEED;
// // }

// //编码器记录路程
// void Car_distance(void)
// {
//     //记录保护编码器
//     Record_encoder=(speed_encoder_L+speed_encoder_R)/2;
//     SystemData.SpeedData.Length+=Record_encoder;

//     if(SystemData.Stop==2)
//     {
//         ImageStatus.Stop_lenth+=Record_encoder;
//     }
//     else
//         ImageStatus.Stop_lenth=0;


//     //记录斑马线的路程
//     if (ImageFlag.Zebra_Flag != 0)  ///检测到车库
//     {
//         ImageStatus.Barn_Lenth += Record_encoder;
//         // beep.gpio_level_set(GPIO_HIGH);
//     }
        
//     else
//     {
//         ImageStatus.Barn_Lenth = 0;
//         // beep.gpio_level_set(GPIO_LOW); 
//     }
        


// }

// //变速控制
// void Car_Speed_Control(void)
// {


//     //直线加速
//     if(ImageStatus.straight_acc==1)
//     {
//         Speed_Goal=ZD_Speed_Goal;
//         // beep.gpio_level_set(GPIO_HIGH);
//     }
//     else
//     {
//         Speed_Goal=WD_Speed_Goal;
//         // beep.gpio_level_set(GPIO_LOW);
//     }




    
//     if(SystemData.Stop==1 ||  ImageStatus.Road_type == Barn_in)  //停车
//     {
//         Speed_Goal=0;
//         Speed_Goal_l=0;
//         Speed_Goal_r=0;
//     }

// }


// // void Speed_BiHuan(void)
// // {
// //     speed_encoder_L = encoder_num_L();
// //     speed_encoder_R = encoder_num_R();


// //     Car_distance();         //编码器记录路程
// //     Car_Speed_Control();     //变速控制 



// //     speed_diff();
// //     Speed_PID_L();
// //     Speed_PID_R();
// //     motor_Duty_R(speed_out_r);
// //     motor_Duty_L(speed_out_l);


// //     // printf("l:%d     ", speed_out_l);
// //     // // printf("    ");
// //     // printf("r:%d\n", speed_out_r);
// //     // printf("%d\n",Garage_Location_Flag);
// //     // motor_Duty_R(1000);
// //     // motor_Duty_L(-1000);
    
// // }

// void Speed_BiHuan(void)
// {
//     speed_encoder_L = encoder_num_L();
//     speed_encoder_R = encoder_num_R();

//     // =============== 新增调试打印区 ===============
//     // 开启这行可观察底盘速度环到底想干嘛
//     printf("Goal L:%d R:%d | Enc L:%d R:%d | Err L:%d R:%d | SteerErr:%d | Det_True:%d\n", 
//            Speed_Goal_l, Speed_Goal_r, speed_encoder_L, speed_encoder_R, speed_erro_L, speed_erro_R, 
//            (ImageStatus.MiddleLine - ImageStatus.Det_True), ImageStatus.Det_True);

//     Car_distance();         // 编码器记录路程
//     Car_Speed_Control();     // 变速控制，更新基础速度Speed_Goal
//     speed_diff();             // 【重点】先算差速，更新左右轮目标速度
//     Speed_PID_L();           // 再执行左右轮PID闭环
//     Speed_PID_R();
//     motor_Duty_R(speed_out_r);
//     motor_Duty_L(speed_out_l);
// }

// // 差速速度调整函数
// void Speed_Adjust_By_Offset(float offset)
// {
//     float KP = 0.3f;  // 方向调整比例系数，可调试

//     float turn = KP * offset;  // 转向量

//     int base_speed = Speed_Goal;  // 当前目标速度，三轮车车型最大速度设定

//     // 左右轮速度由基准速度和偏差计算
//     Speed_Goal_l = base_speed - turn;
//     Speed_Goal_r = base_speed + turn;

//     // 限幅保护，防止速度超过范围或为负
//     if (Speed_Goal_l > MAX_SPEED) Speed_Goal_l = MAX_SPEED;
//     if (Speed_Goal_l < 0)         Speed_Goal_l = 0;

//     if (Speed_Goal_r > MAX_SPEED) Speed_Goal_r = MAX_SPEED;
//     if (Speed_Goal_r < 0)         Speed_Goal_r = 0;
// }

// // 电机紧急停止函数
// void emergency_stop() {
//     motor_Duty_R(0);    // 右电机停转
//     motor_Duty_L(0);    // 左电机停转
//     motor_R.atim_pwm_set_active_low_duty(0); // 低有效板停车，保持PWM使能
//     motor_L.atim_pwm_set_active_low_duty(0);
//     // stree_M.atim_pwm_set_duty(170000);  // 舵机归中
//     printf("程序终止，电机已停止。\n");
// }

// // 信号处理函数（捕获 Ctrl+C 等终止信号）
// void signal_handler(int signum) {
//     exit_flag = true;    // 触发子线程退出
//     emergency_stop();    // 立即停止电机
//     exit(signum);        // 终止进程
// }

/**************************
 * @brief 安全停机函数：彻底关停所有电机、复位硬件
 **************************/
// void SafeMotorStop(void)
// {
//     // 1. 先锁死目标速度为0
//     Speed_Goal = 0;
//     Speed_Goal_l = 0;
//     Speed_Goal_r = 0;

//     // 2. 清零PID参数和输出
//     speed_out_l = 0;
//     speed_out_r = 0;
//     speed_erro_L = 0;
//     speed_erro_R = 0;
//     speed_last_erro_L = 0;
//     speed_last_erro_R = 0;

//     // 3. 强制设置电机PWM占空比为0，停止输出
//     motor_Duty_L(0);
//     motor_Duty_R(0);

//     // 4. 失能PWM外设，彻底关闭输出
//     motor_R.atim_pwm_set_active_low_duty(0);
//     motor_L.atim_pwm_set_active_low_duty(0);

//     // 5. 复位电机方向引脚，拉低电平
//     motor_R_Dir.gpio_level_set(GPIO_HIGH);
//     motor_L_Dir.gpio_level_set(GPIO_HIGH);

//     // 6. 关闭蜂鸣器
//     beep.gpio_level_set(GPIO_LOW);

//     // 7. 打印停机日志
//     printf("\n【安全停机】电机已完全停止，程序即将退出\n");
// }

// /**************************
//  * @brief 信号处理函数：捕获终止信号，设置退出标志
//  **************************/
// void SignalHandler(int signal)
// {
//     switch(signal)
//     {
//         case SIGINT:  // Ctrl+C 触发的信号
//             printf("\n【收到终止信号】准备安全停机...\n");
//             exit_flag.store(true); // 设置退出标志
//             break;
//         case SIGTERM: // 系统终止信号
//             printf("\n【收到终止信号】准备安全停机...\n");
//             exit_flag.store(true); // 设置退出标志
//             break;
//         case SIGSEGV: // 段错误异常信号（兜底）
//             printf("\n【收到终止信号】准备安全停机...\n");
//             exit_flag.store(true); // 设置退出标志
//             break;
//         default:
//             break;
//     }
// }
