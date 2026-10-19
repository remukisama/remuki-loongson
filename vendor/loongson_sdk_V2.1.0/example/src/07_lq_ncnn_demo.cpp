#include "lq_all_demo.hpp"
#include <opencv2/opencv.hpp>
#include <ncnn/net.h>

/********************************************************************************
 * @file    lq_ncnn_demo.cpp
 * @brief   NCNN 测试.
 * @author  龙邱科技-006
 * @date    2026-01-10
 * @version V2.1.0
 * @note    适用与龙芯 2K0300/0301 平台.
 *          本 demo 实现 NCNN 功能，用于测试 NCNN 模型的基本功能.
 ********************************************************************************/

// 获取当前时间字符串（格式：YYYY-MM-DD HH:MM:SS）
static std::string get_time_str(void)
{
    // 时间获取对象
    static lq_ntp g_ntp;
    return g_ntp.get_local_time_str();
}

void lq_ncnn_demo(void)
{
    try {
        std::cout << "========== NCNN 环境自检(无模型) ==========" << std::endl;

        // 创建 ncnn::Net
        ncnn::Net net;
        net.opt.num_threads = 2;
        net.opt.use_vulkan_compute = false;  // 基础检查

        // 创建测试输入张量，验证 Mat 内存与基础操作
        ncnn::Mat input(8, 8, 3); // w, h, c
        for (int c = 0; c < input.c; ++c) {
            float* ptr = input.channel(c);
            for (int i = 0; i < input.w * input.h; ++i) {
                ptr[i] = static_cast<float>(c * 100 + i);
            }
        }

        ncnn::Mat input_clone = input.clone();
        float checksum = 0.0f;
        for (int c = 0; c < input_clone.c; ++c) {
            const float* ptr = input_clone.channel(c);
            for (int i = 0; i < input_clone.w * input_clone.h; ++i) {
                checksum += ptr[i];
            }
        }

        // 创建提取器
        ncnn::Extractor ex = net.create_extractor();

        // 未加载模型时，输入层不存在，返回非 0 属于预期行为
        int ret = ex.input("data", input_clone);

        std::cout << "Mat 维度: w=" << input_clone.w
                  << ", h=" << input_clone.h
                  << ", c=" << input_clone.c << std::endl;
        std::cout << "Mat 校验和: " << checksum << std::endl;
        std::cout << "Extractor input 返回值(无模型预期非0): " << ret << std::endl;

        if (ret != 0) {
            std::cout << "[PASS] ncnn 基础环境可用（头文件/链接/运行时对象均正常）。" << std::endl;
            return;
        }

        std::cout << "[WARN] 未加载模型却返回0，建议进一步检查 ncnn 版本与调用逻辑。" << std::endl;
        return;
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] ncnn 环境自检异常: " << e.what() << std::endl;
        return;
    } catch (...) {
        std::cerr << "[FAIL] ncnn 环境自检出现未知异常" << std::endl;
        return;
    }
}
void lq_ncnn_photo_demo(void)
{
    printf("========================================\n");
    printf("       NCNN 图像分类推理示例\n");
    printf("========================================\n\n");

    // ==================== 配置区域 ====================
    // 测试图片路径（修改为你的图片路径）
    std::string test_image_path = "test.jpg";
    
    // 模型配置
    std::string model_param = "tiny_classifier_fp32.ncnn.param";
    std::string model_bin   = "tiny_classifier_fp32.ncnn.bin";
    int input_width    = 96;
    int input_height   = 96;
    
    // 类别标签（顺序必须与训练时一致）
    std::vector<std::string> labels = {"supplies", "vehicle", "weapon"};
    
    // 归一化参数（ImageNet标准）
    float mean_vals[3] = {123.675f, 116.28f, 103.53f};
    float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};
    // =================================================

    // 创建NCNN对象并配置
    LQ_NCNN ncnn;
    ncnn.SetModelPath(model_param, model_bin);
    ncnn.SetInputSize(input_width, input_height);
    ncnn.SetLabels(labels);
    ncnn.SetNormalize(mean_vals, norm_vals);

    // 初始化模型
    printf("[%s] 正在加载模型...\n", get_time_str().c_str());
    if (!ncnn.Init()) {
        printf("[%s] 模型加载失败!\n", get_time_str().c_str());
        return ;
    }
    printf("[%s] 模型加载成功!\n\n", get_time_str().c_str());

    // 读取测试图片
    printf("[%s] 读取图片: %s\n", get_time_str().c_str(), test_image_path.c_str());
    cv::Mat image = cv::imread(test_image_path);

    if (image.empty()) {
        printf("无法读取图片: %s\n", test_image_path.c_str());
        printf("请确保图片文件存在\n");
        return ;
    }
    printf("[%s] 图片尺寸: %d x %d\n\n", get_time_str().c_str(), image.cols, image.rows);

    // 注意: OpenCV读取的是BGR格式，但在推理时会自动转换为RGB格式以匹配训练时的输入
    // 训练使用PIL读取的RGB格式，因此需要色彩空间转换

    // 推理
    printf("[%s] 开始推理...\n", get_time_str().c_str());
    auto start = std::chrono::high_resolution_clock::now();
    std::string result = ncnn.Infer(image);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // 输出结果
    printf("\n========================================\n");
    printf("推理结果: %s\n", result.c_str());
    printf("推理耗时: %ld ms\n", duration.count());
    printf("========================================\n");

    return ;
}

