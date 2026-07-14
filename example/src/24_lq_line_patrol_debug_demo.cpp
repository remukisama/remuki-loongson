#include "main.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>

/********************************************************************************
 * @file    24_lq_line_patrol_debug_demo.cpp
 * @brief   巡线算法调试图传 Demo (使用UDP).
 * @author  龙邱科技 / GitHub Copilot
 * @date    2026-04-14
 * @note    该 Demo 将巡线的二值化图像、左右边线、中线、帧率、赛道状态等信息通过UDP图传给上位机
 ********************************************************************************/

// =====================================================
// 配置参数 - 根据需要修改
// =====================================================
static const std::string TARGET_IP    = "192.168.255.22";  // 目标PC IP地址
static const uint16_t    TARGET_PORT  = 8080;             // 目标UDP端口
static const uint8_t     JPEG_QUALITY = 50;               // 压缩质量(1~100)

extern uint8_t img1[60][80];
extern ImageDealDatatypedef ImageDeal[60];
extern ImageStatustypedef ImageStatus;

// 格式化输出路况类型 (注: OpenCV原生putText不支持直接渲染中文，故图传画面补充拼音，中文写在代码注释中)
static std::string get_road_type_string(int road_type) {
    switch(road_type) {
        case 0: return "Zero (ChuShi)";
        case 1: return "Normol (ChangGui)";
        case 2: return "Straight (ZhiDao)";
        case 3: return "Cross (ShiZi)";
        case 4: return "Ramp (PoDao)";
        case 5: return "LeftCirque (ZuoHuan)";
        case 6: return "RightCirque (YouHuan)";
        case 7: return "Cross_ture (ZhenShiZi)";
        case 8: return "Barn_in (BanMaXian)";
        // 若有更多枚举请自行添加，这里默认转字符
        default: return "Type_" + std::to_string(road_type);
    }
}

static std::string get_ring_state_string(int flag) {
    switch(flag) {
        case 0: return "0: Not in Ring (Wu)";
        case 1: return "1: Pre-Ring (ZhunBei)";
        case 2: return "2: Enter cut (QieKou)";
        case 3: return "3: Check (PanDuan)";
        case 4: return "4: Wait (DengDai)";
        case 5: return "5: Turning in (ZhuanRu)";
        case 6: return "6: Inside Ring (HuanNei)";
        case 7: return "7: Search Exit (ZhaoChuKou)";
        case 8: return "8: Force Exit (XianChu)";
        case 9: return "9: Exit Wait (QueRen)";
        default: return "State_" + std::to_string(flag);
    }
}

void lq_line_patrol_debug_demo(void)
{
    printf("=========================================\r\n");
    printf("  Line Patrol Debug & UDP Stream Demo    \r\n");
    printf("=========================================\r\n");
    printf("Target IP:   %s\r\n", TARGET_IP.c_str());
    printf("Target Port: %d\r\n", TARGET_PORT);
    printf("=========================================\r\n");

    // 1. 初始化 UDP 和图像结构
    lq_udp_client udp_client;
    udp_client.udp_client_init(TARGET_IP, TARGET_PORT);

    Data_Settings();              // 初始化图像参数
    if (image_cv_Init() == -1) {  // 初始化摄像头
        printf("ERROR: Failed to open camera!\r\n");
        return;
    }

    uint32_t frame_count = 0;
    float current_fps = 0.0f;
    uint32_t skip_counter = 0; // 新增跳帧计数器
    auto last_time = std::chrono::high_resolution_clock::now();

    printf("Start line patrol stream... Press Ctrl+C to stop\r\n");

    while (true) 
    {
        // 2. 获取一帧图像并二值化存入 img1 (全速运行)
        if (image_cv_zip() == -1) {
            printf("Read frame error or stopped\n");
            break;
        }

        // 3. 执行巡线算法处理 (全速运行)
        imageprocess();

        // 记录真实的算法运行帧率
        frame_count++;
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
        if (elapsed >= 1000) {
            current_fps = frame_count * 1000.0f / elapsed;
            frame_count = 0;
            last_time = now;
        }

        // ============================================
        // CPU算力已得到解放(<20%)，不再跳帧，每帧都传输！
        // ============================================

        // 4. 创建一帧用于显示的彩色RGB图像（原始分辨率60行，80列）
        // img1: 60行 (y), 80列 (x)
        cv::Mat debug_frame(60, 80, CV_8UC3);

        for (int y = 0; y < 60; y++) {
            for (int x = 0; x < 80; x++) {
                // 背景：黑；赛道(白) 赋予 255
                if (img1[y][x] == 1) { 
                    debug_frame.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
                } else {
                    debug_frame.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // 黑底
                }
            }
            
            // 绘制找到的/补出的边线和中线（按点绘制颜色不同）
            // 左边线：红色
            if (ImageDeal[y].IsLeftFind == 'T' || ImageDeal[y].IsLeftFind == 'H') {
                int lx = ImageDeal[y].LeftBorder;
                if (lx >= 0 && lx < 80) {
                    debug_frame.at<cv::Vec3b>(y, lx) = cv::Vec3b(0, 0, 255); 
                }
            }
            // 右边线：绿色
            if (ImageDeal[y].IsRightFind == 'T' || ImageDeal[y].IsRightFind == 'H') {
                int rx = ImageDeal[y].RightBorder;
                if (rx >= 0 && rx < 80) {
                    debug_frame.at<cv::Vec3b>(y, rx) = cv::Vec3b(0, 255, 0);
                }
            }
            // 中线：蓝色
            int cx = ImageDeal[y].Center;
            if (cx >= 0 && cx < 80) {
                debug_frame.at<cv::Vec3b>(y, cx) = cv::Vec3b(255, 0, 0);
            }
        }

        // 5. 放大图像以便上位机观察，否则 80x60 太小看不清字 (插值使用最近邻即可保留像素感)
        cv::Mat display_frame;
        cv::resize(debug_frame, display_frame, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);

        // 7. 在图像上写入调试字符串信息
        char text_buf[64];
        std::string road_str = get_road_type_string(ImageStatus.Road_type);
        
        sprintf(text_buf, "Road: %s", road_str.c_str());
        cv::putText(display_frame, text_buf, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 165, 255), 2);
        
        sprintf(text_buf, "FPS: %.1f", current_fps);
        cv::putText(display_frame, text_buf, cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
        
        sprintf(text_buf, "SRC: 80x60 (OUT: 320x240)");
        cv::putText(display_frame, text_buf, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        
        sprintf(text_buf, "RingState: %s", get_ring_state_string(ImageFlag.image_element_rings_flag).c_str());
        cv::putText(display_frame, text_buf, cv::Point(10, 95), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

        // 8. 通过 UDP 发送最终图像
        ssize_t sent = udp_client.udp_send_image(display_frame, JPEG_QUALITY);
        if (sent < 0) {
            // printf("ERROR: Failed to send image\r\n");
        }
    }
}

// ==========================================
// 非阻塞版：发送一帧中线叠加图（供主循环调用）
// ==========================================
void lq_udp_center_line_send_one_frame(lq_udp_client &client, int quality)
{
    cv::Mat frame(60, 80, CV_8UC3);
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 80; ++x) {
            frame.at<cv::Vec3b>(y, x) = img1[y][x] ? cv::Vec3b(235, 235, 235)
                                                   : cv::Vec3b(0, 0, 0);
        }
        int cx = ImageDeal[y].Center;
        if (cx >= 0 && cx < 80) frame.at<cv::Vec3b>(y, cx) = cv::Vec3b(255, 0, 0);
        int lx = ImageDeal[y].LeftBorder;
        int rx = ImageDeal[y].RightBorder;
        if (lx >= 0 && lx < 80) frame.at<cv::Vec3b>(y, lx) = cv::Vec3b(0, 0, 255);
        if (rx >= 0 && rx < 80) frame.at<cv::Vec3b>(y, rx) = cv::Vec3b(0, 255, 0);
    }

    cv::Mat enlarged;
    cv::resize(frame, enlarged, cv::Size(320, 240), 0, 0, cv::INTER_NEAREST);
    cv::Mat display_frame(300, 320, CV_8UC3, cv::Scalar(0, 0, 0));
    enlarged.copyTo(display_frame(cv::Rect(0, 0, 320, 240)));
    char text_buf[192];
    std::snprintf(text_buf, sizeof(text_buf),
                  "Road:%d R:%d S:%d SF:%d/%dms A:%d Y:%d",
                  static_cast<int>(ImageStatus.Road_type),
                  ImageFlag.image_element_rings,
                  ImageFlag.image_element_rings_flag,
                  g_ring_stage_frames,
                  g_ring_stage_age_ms,
                  g_ring_active_frames,
                  g_ring_turn_yaw_mdeg.load(std::memory_order_relaxed) / 1000);
    cv::putText(display_frame, text_buf, cv::Point(6, 257),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 165, 255), 1);
    std::snprintf(text_buf, sizeof(text_buf),
                  "det:%d off:%d ll/rl:%d/%d",
                  ImageStatus.Det_True, ImageStatus.OFFLine,
                  ImageStatus.Left_Line, ImageStatus.Right_Line);
    cv::putText(display_frame, text_buf, cv::Point(6, 275),
                cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(255, 255, 0), 1);
    std::snprintf(text_buf, sizeof(text_buf),
                  "L:b%d g%d x%d c%d  R:b%d g%d x%d c%d",
                  dbg_el_l_block, dbg_el_l_geom, dbg_el_l_loss, dbg_el_l_confirm,
                  dbg_el_r_block, dbg_el_r_geom, dbg_el_r_loss, dbg_el_r_confirm);
    cv::putText(display_frame, text_buf, cv::Point(6, 293),
                cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(255, 255, 255), 1);
    client.udp_send_image(display_frame, quality);
}
