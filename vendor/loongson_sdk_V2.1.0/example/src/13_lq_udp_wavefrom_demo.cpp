#include "lq_all_demo.hpp"

/********************************************************************************
 * @file    lq_udp_wavefrom_demo.cpp
 * @brief   UDP 传输波形绘制测试.
 * @author  龙邱科技-012
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 UDP 传输数据，并使用上位机绘制波形.
 ********************************************************************************/

// =====================================================
// 配置参数 - 根据需要修改
// =====================================================
// 目标IP地址（UDP接收端，也就是PC端IP）
const std::string TARGET_IP    = "192.168.43.98";
// UDP目标端口
const uint16_t    TARGET_PORT  = 8080;

/********************************************************************************
 *  @brief   UDP 传输波形绘制测试.
 *  @param   none.
 *  @return  none.
 *  @note    测试内容为 UDP 传输数据，并使用上位机绘制波形.
 *! @note    使用时需搭配对应上位机 LoongHost.exe，用于接收并绘制波形.
 ********************************************************************************/
void lq_udp_wavefrom_demo(void)
{
    printf("=========================================\n");
    printf("  UDP 编码器数据传输\n");
    printf("=========================================\n");
    printf("Target IP:   %s\n", TARGET_IP.c_str());
    printf("Target Port: %d\n", TARGET_PORT);
    printf("=========================================\n");

    // 初始化UDP客户端
    lq_udp_client udp_client;
    udp_client.udp_client_init(TARGET_IP, TARGET_PORT);
    printf("UDP client initialized\n");

    // 初始化编码器 (ENC_PWM0_PIN64, PIN_73)
    // 根据实际硬件接线修改引脚
    ls_encoder_pwm enc1(ENC_PWM0_PIN64, PIN_72);
    ls_encoder_pwm enc2(ENC_PWM1_PIN65, PIN_73);  // 如果有第二个编码器
    printf("Encoder initialized\n");

    printf("Start streaming... Press Ctrl+C to stop\n");

    while (ls_system_running.load()) {
        // ===================== 获取并发送编码器值 =====================
        // 读取编码器值
        float ch1 = enc1.encoder_get_count();
        float ch2 = enc2.encoder_get_count();  // 如果有第二个编码器

        // 格式化字符串: ch1:1.23,ch2:4.56
        char encoder_str[64];
        // 如果只有一个编码器
        // snprintf(encoder_str, sizeof(encoder_str), "ch1:%.2f", ch1);
        // 如果有两个编码器
        snprintf(encoder_str, sizeof(encoder_str), "ch1:%7.2f,ch2:%7.2f", ch1, ch2);

        // 发送编码器数据
        udp_client.udp_send_string(encoder_str);

        // ===================== 打印状态 =====================
        printf("Encoder: %s\n", encoder_str);
        usleep(100*100);
    }
}
