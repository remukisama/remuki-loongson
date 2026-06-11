#include <iostream>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "lq_camera_ex.hpp"

void lq_camera_video_demo(void)
{
    std::cout << "====== 龙芯2K300/2K301 摄像头视频录制测试 ======" << std::endl;

    // 使用 LQ_CAMERA_0CPU_MJPG 降低 CPU 占用
    lq_camera_ex camera(320, 240, 180, LQ_CAMERA_0CPU_MJPG);

    std::cout << "正在启动摄像头采集..." << std::endl;
    if (camera.start_collect() != 0)
    {
        std::cerr << "摄像头启动失败！请检查摄像头是否连接。" << std::endl;
        return;
    }

    std::cout << "正在配置手动曝光..." << std::endl;
    if (!camera.set_exposure_manual(55))
    {
        std::cerr << "警告: 设置手动曝光失败" << std::endl;
    }
    usleep(500 * 1000);

    // 原子标志位，由 stdin 线程设置、主循环读取
    std::atomic<bool> exit_flag(false);
    std::atomic<bool> toggle_flag(false);

    // 独立线程读取键盘输入，不阻塞主循环
    std::thread input_thread([&]() {
        while (!exit_flag.load())
        {
            std::string line;
            std::getline(std::cin, line);
            if (line == "q" || line == "Q")
            {
                exit_flag.store(true);
                toggle_flag.store(true); // 让主循环退出录制状态
            }
            else
            {
                toggle_flag.store(true);
            }
        }
    });

    std::cout << "\n------------------------------------------" << std::endl;
    std::cout << "  按回车键 (Enter) 开始录制。" << std::endl;
    std::cout << "  再次按回车键停止录制并保存视频。" << std::endl;
    std::cout << "  输入 'q' 并回车退出程序。" << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "\n摄像头就绪 | 分辨率: "
              << camera.get_camera_width() << "x" << camera.get_camera_height()
              << " | FPS: " << camera.get_camera_fps() << std::endl;
    std::cout << "等待指令: " << std::flush;

    bool recording    = false;
    cv::VideoWriter   writer;
    int               video_index   = 0;
    int               frame_count   = 0;

    // FPS 统计
    using clock = std::chrono::steady_clock;
    auto last_fps_time  = clock::now();
    int  fps_frame_cnt  = 0;

    while (!exit_flag.load())
    {
        // 处理键盘指令
        if (toggle_flag.exchange(false))
        {
            if (exit_flag.load()) break; // q 退出

            if (!recording)
            {
                // ========== 开始录制 ==========
                std::string filename = "video_" + std::to_string(video_index++) + "_320x240.avi";
                int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

                writer.open(filename, fourcc, 30.0, cv::Size(320, 240));
                if (!writer.isOpened())
                {
                    // MJPG 不可用时尝试默认编码器
                    std::cout << "\n[MJPG 编码器不可用, 尝试默认编码器...]" << std::endl;
                    writer.open(filename, -1, 30.0, cv::Size(320, 240));
                }
                if (!writer.isOpened())
                {
                    std::cerr << "\n[错误] VideoWriter 打开失败！此 OpenCV 版本可能缺少 videoio 编码后端。" << std::endl;
                    std::cout << "\n等待指令: " << std::flush;
                    continue;
                }

                frame_count    = 0;
                fps_frame_cnt  = 0;
                last_fps_time  = clock::now();
                recording      = true;

                std::cout << "\n[开始录制] → " << filename << std::endl;
                std::cout << "按回车键停止录制..." << std::flush;
            }
            else
            {
                // ========== 停止录制 ==========
                recording = false;
                writer.release();
                std::cout << "\n[录制停止] 共 " << frame_count
                          << " 帧已保存到视频文件。" << std::endl;
                std::cout << "\n等待指令: " << std::flush;
                continue;
            }
        }

        // 录制循环
        if (recording)
        {
            cv::Mat frame = camera.get_frame_raw();
            if (frame.empty())
            {
                std::cerr << "\n获取图像帧失败！停止录制。" << std::endl;
                recording = false;
                writer.release();
                std::cout << "\n等待指令: " << std::flush;
                continue;
            }

            // 如果摄像头物理倒置，翻转 180°
            cv::flip(frame, frame, -1);

            writer.write(frame);
            frame_count++;
            fps_frame_cnt++;

            // 每秒打印一次录制状态
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               clock::now() - last_fps_time)
                               .count();
            if (elapsed >= 1000)
            {
                std::cout << "\r[录制中] " << frame_count << " 帧 | "
                          << "实际帧率: ~" << fps_frame_cnt << " fps   "
                          << " (按回车停止)" << std::flush;
                fps_frame_cnt = 0;
                last_fps_time = clock::now();
            }
        }
        else
        {
            // 空闲等待，降低 CPU 占用
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // 退出清理
    if (recording)
    {
        writer.release();
    }
    camera.stop_collect();

    input_thread.join();
    std::cout << "\n程序已退出。" << std::endl;
}
