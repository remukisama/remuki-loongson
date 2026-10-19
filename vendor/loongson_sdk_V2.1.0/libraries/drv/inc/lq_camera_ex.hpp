/************************************************************************************************************************
 * @file      : lq_camera_ex
 * @brief     : 龙芯2K0300/2K301 板卡外设驱动 - 摄像头模块（增强版）
 * @author    : 龙邱科技
 * @email     : chiusir@163.com
 * @compiler  : Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
 * @platform  : 龙芯2K0300久久派子母板和北京龙邱智能科技龙邱2K301双龙mini派子母板
 * @version   : 版权所有，单位使用请先联系授权
 * @reference : 相关信息参考下列地址
 *              公司官网链接：http://www.lqist.cn
 *              淘宝店铺链接：http://longqiu.taobao.com
 *              程序配套视频：https://space.bilibili.com/95313236
 * @reference : 参考项目链接：
 *              1. https://gitee.com/Wuwu129/SmartCar_99Pai_OpenSource
 * @license   : 本项目参考 Wuwu 开源库实现，遵循 GNU 通用公共许可证第3版（GPL-3.0）或其后续版本
 *              1. 可自由使用、修改、分发本代码，分发时需随附本许可声明及 GPL-3.0 完整许可证副本
 *              2. 本软件按“原样”提供，不做任何明示/暗示保证，使用风险自负
 *              3. 第三方组件版权与许可依其自身 LICENSE 文件为准
 *              许可证原文：https://www.gnu.org/licenses/gpl-3.0.html
 * @update    : 2025-03-12
 ************************************************************************************************************************/
#ifndef __LQ_CAMERA_EX_HPP
#define __LQ_CAMERA_EX_HPP

#include <iostream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <mutex>
#include <stdint.h>
#include <sys/ioctl.h>
#include <memory>

#include <opencv2/opencv.hpp>

/****************************************************************************************************
 * @brief   宏定义
 ****************************************************************************************************/

#define LQ_CAMERA_PATH          ( "/dev/video0" )

/****************************************************************************************************
 * @brief   枚举定义
 ****************************************************************************************************/

// 摄像头获取图像方式枚举
typedef enum
{
    LQ_CAMERA_HIGH_MJPG = 0x00, // 高帧率
    LQ_CAMERA_0CPU_MJPG,        // 低 CPU 占用
} lq_camera_format_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class lq_camera_ex
{
public:
    // 有参构造函数构造函数
    explicit lq_camera_ex(uint16_t _width, uint16_t _height, uint16_t _fps, /* 摄像头宽高和帧率设置 */
                          lq_camera_format_t _fmt = LQ_CAMERA_HIGH_MJPG,    /* 获取图像方式 */
                          const std::string _path = LQ_CAMERA_PATH);        /* 摄像头设备路径 */
    
    lq_camera_ex(const lq_camera_ex&) = delete;             // 禁用拷贝
    lq_camera_ex& operator=(const lq_camera_ex&) = delete;  // 禁用赋值
    lq_camera_ex(lq_camera_ex&&) = delete;                  // 禁用移动构造函数
    lq_camera_ex& operator=(lq_camera_ex&&) = delete;       // 禁用移动赋值运算符

    // 析构函数
    ~lq_camera_ex();

public:
    /********************************************************************************
     * @brief   初始化摄像头.
     * @param   _width  : 图像宽度.
     * @param   _height : 图像高度.
     * @param   _fps    : 帧率.
     * @param   _format : 图像格式.
     * @param   _path   : 设备路径.
     * @return  成功返回 0, 失败返回负数.
     * @note    如果已经初始化, 会先释放资源再重新初始化，创建变量时会默认运行.
     ********************************************************************************/
    int init(uint16_t _width, uint16_t _height, uint16_t _fps, 
             lq_camera_format_t _format = LQ_CAMERA_HIGH_MJPG, 
             const std::string _path = LQ_CAMERA_PATH);

    /********************************************************************************
     * @brief   开始采集图像.
     * @param   none.
     * @return  成功返回 0, 失败返回负数.
     * @note    创建变量成功时会默认开始采集.
     ********************************************************************************/
    int start_collect();

    /********************************************************************************
     * @brief   停止采集图像.
     * @param   none.
     * @return  成功返回 0, 失败返回负数.
     ********************************************************************************/
    int stop_collect();

public:
    /********************************************************************************
     * @brief   获取一帧原始图像.
     * @return  成功返回原始图像, 失败返回空cv::Mat.
     * @note    调用该函数会从摄像头获取一帧原始图像，并返回.
     ********************************************************************************/
    cv::Mat get_frame_raw();

    /********************************************************************************
     * @brief   获取一帧灰度图像.
     * @return  成功返回灰度图像, 失败返回空cv::Mat.
     * @note    调用该函数会从摄像头获取一帧原始图像，并转为灰度后返回.
     ********************************************************************************/
    cv::Mat get_frame_gray();

    /********************************************************************************
     * @brief   获取一帧原始图像和灰度图像.
     * @param   [out] raw    原始图像.
     * @param   [out] gray   灰度图像.
     * @return  成功返回true, 否则返回false.
     * @note    调用该函数会从摄像头获取一帧原始图像，并转为灰度后返回原始图像和灰度图像.
     ********************************************************************************/
    bool get_frame_raw_gray(cv::Mat &raw, cv::Mat &gray);

    /********************************************************************************
     * @brief   设置摄像头的手动曝光.
     * @param   expo : 曝光值.
     * @return  成功返回true, 否则返回false.
     * @note    建议曝光值在[20, 200]之间，当然可以超出该范围.
     * @note    摄像头断电重启后，默认为自动曝光模式.
     ********************************************************************************/
    bool set_exposure_manual(int16_t expo);

    /********************************************************************************
     * @brief   保存一帧图片为jpg文件.
     * @param   frame : 图片数据.
     * @param   filename : 文件名.
     * @return  成功返回true, 否则返回false.
     ********************************************************************************/
    bool save_image_picture(const cv::Mat &frame, const std::string &filename);

public:
    // 获取摄像头信息
    uint16_t get_camera_width()  const; // 摄像头宽度
    uint16_t get_camera_height() const; // 摄像头高度
    uint16_t get_camera_fps()    const; // 摄像头帧率
    
    // 检查摄像头是否已打开
    bool is_cam_opened() const;

private:
    struct lq_camera_ex_Impl;                   // 类内结构体
    std::unique_ptr<lq_camera_ex_Impl> pImpl;   // 使用智能指针管理
};

#endif
