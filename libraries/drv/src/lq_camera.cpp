#include "lq_camera.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @brief   摄像头类有参构造函数.
 * @param   _width  : 摄像头分辨率宽.
 * @param   _height : 摄像头分辨率高.
 * @param   _fps    : 摄像头帧率.
 * @param   _dev_id : 摄像头设备号, 默认为 0.
 * @return  none.
 * @example ls_camera cam(160, 120, 180);
 * @note    none.
 ********************************************************************************/
lq_camera::lq_camera(uint16_t _width, uint16_t _height, uint16_t _fps, int _dev_id)
{
#if !LQ_OPENCV_AVAILABLE
    lq_log_error("OpenCV related function library not added");
#else
    this->dev_id = _dev_id;
    this->open(_width, _height, _fps);
#endif
}

/********************************************************************************
 * @brief   摄像头类的析构函数.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    变量生命周期结束自动调用.
 ********************************************************************************/
lq_camera::~lq_camera()
{
    this->close();
    lq_log_info("摄像头资源已安全释放!");
}

/********************************************************************************
 * @brief   打开摄像头.
 * @param   _width  : 摄像头分辨率宽.
 * @param   _height : 摄像头分辨率高.
 * @param   _fps    : 摄像头帧率.
 * @return  none.
 * @example cam.open(160, 120, 180);
 * @note    创建变量时内部会自动调用, 一般情况下无需再次调用.
 ********************************************************************************/
bool lq_camera::open(uint16_t _width, uint16_t _height, uint16_t _fps)
{
    std::lock_guard<std::mutex> lock(this->mtx);
#if LQ_OPENCV_AVAILABLE
    // 关闭已打开的摄像头
    if (this->is_open)
    {
        this->cap.release();
        this->is_open = false;
    }
    // 打开摄像头设备
    this->cap = cv::VideoCapture(this->dev_id);
    if (!this->cap.isOpened())
    {
        lq_log_error("打开摄像头失败!");
        return false;
    }
    // 设置视频流编码器
    this->cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // 设置摄像头图像宽高和帧率
    this->cap.set(cv::CAP_PROP_FRAME_WIDTH, _width);
    this->cap.set(cv::CAP_PROP_FRAME_HEIGHT, _height);
    this->cap.set(cv::CAP_PROP_FPS, _fps);
    // 获取实际参数
    double actual_width  = this->cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = this->cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actual_fps    = this->cap.get(cv::CAP_PROP_FPS);
    // 获取摄像头图像宽高和帧率
    this->width  = static_cast<uint16_t>(actual_width);
    this->height = static_cast<uint16_t>(actual_height);
    this->fps    = static_cast<uint16_t>(actual_fps);
    // 打印实际参数
    if (actual_width != _width || actual_height != _height)
    {
        lq_log_warn("Camera resolution adjusted: set(%ux%u) -> actual(%ux%u)", _width, _height, this->width, this->height);
    }
    if (actual_fps != _fps)
    {
        lq_log_warn("Camera FPS adjusted: set(%u) -> actual(%u)", _fps, this->fps);
    }
    // 设置摄像头状态
    this->is_open = true;
    lq_log_info("Camera opened successfully (dev_id:%d, %ux%u@%ufps)", this->dev_id, this->width, this->height, this->fps);
#else
    lq_log_error("Opencv 库未加载!\n");
#endif
    return true;
}

/********************************************************************************
 * @brief   关闭摄像头.
 * @param   none.
 * @return  none.
 * @example cam.close();
 * @note    析构函数中会自动调用, 一般情况下无需再次调用.
 ********************************************************************************/
void lq_camera::close()
{
    std::lock_guard<std::mutex> lock(this->mtx);
#if LQ_OPENCV_AVAILABLE
    // 关闭摄像头设备
    if (this->is_open)
        this->cap.release();
    this->frame.release();
    this->gray_frame.release();
    this->binary_frame.release();
#endif
    // 设置摄像头状态
    this->is_open = false;
    this->dev_id  = -1;
    this->width   = 0;
    this->height  = 0;
    this->fps     = 0;
}

#if LQ_OPENCV_AVAILABLE

/********************************************************************************
 * @brief   获取摄像头最新一帧图像.
 * @param   none.
 * @return  成功返回真, 失败返回假.
 * @example none.
 * @note    内部调用.
 ********************************************************************************/
bool lq_camera::read_new_frame()
{
    if (!this->is_open || !this->cap.isOpened())
    {
        lq_log_error("Camera not opened when reading frame");
        return false;
    }
    // 获取原始图像
    if (!this->cap.read(this->frame))
    {
        lq_log_error("Error read frame from camera (dev_id:%d)", this->dev_id);
        this->frame.release();
        return false;
    }
    return true;
}

/********************************************************************************
 * @brief   判断获取到的图像是否有效.
 * @param   mat : 需要判断的图像.
 * @return  成功返回真, 否则返回假.
 * @example none.
 * @note    内部调用.
 ********************************************************************************/
bool lq_camera::is_mat_valid(const cv::Mat& mat) const
{
    return !mat.empty() && mat.data != nullptr;
}

/********************************************************************************
 * @brief   获取原始图像.
 * @param   none.
 * @return  原始图像.
 * @example cv::Mat raw_frame = cam.get_raw_frame();
 * @note    none.
 ********************************************************************************/
cv::Mat lq_camera::get_raw_frame()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    if (this->read_new_frame() && this->is_mat_valid(this->frame))
    {
        // 返回克隆体，避免外部修改成员变量
        return this->frame.clone();
    }
    return cv::Mat();
}

/**********************************************************************  **********
 * @brief   获取灰度图像.
 * @param   none.
 * @return  灰度图像.
 * @example cv::Mat gray_frame = cam.get_gray_frame();
 * @note    none.
 ********************************************************************************/
cv::Mat lq_camera::get_gray_frame()
{
    std::lock_guard<std::mutex> lock(this->mtx);
    if (!this->read_new_frame())
    {
        lq_log_error("Camera not opened when reading frame");
        return cv::Mat();
    }
    // 转灰度
    cv::cvtColor(this->frame, this->gray_frame, cv::COLOR_BGR2GRAY);
    if (this->is_mat_valid(this->gray_frame))
    {
        // 返回克隆体，避免外部修改成员变量
        return this->gray_frame.clone();
    }
    return cv::Mat();
}

/********************************************************************************
 * @brief   获取二值图像.
 * @param   none.
 * @return  二值图像.
 * @example cv::Mat binary_frame = cam.get_binary_frame();
 * @note    none.
 ********************************************************************************/
cv::Mat lq_camera::get_binary_frame(double _thresh, double _max)
{
    std::lock_guard<std::mutex> lock(this->mtx);
    if (!this->read_new_frame())
    {
        lq_log_error("Camera not opened when reading frame");
        return cv::Mat();
    }
    // 先转灰度
    cv::cvtColor(this->frame, this->gray_frame, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(this->gray_frame, this->blur_frame, cv::Size(3, 3), 0);  // 3x3模糊，去噪神器
    // 二值化
    cv::threshold(this->blur_frame, this->binary_frame, _thresh, _max, cv::THRESH_BINARY);
    // 1. 转 HSV
    cv::Mat hsv;
    cv::cvtColor(this->frame, hsv, cv::COLOR_BGR2HSV);

    // ==============================================
    // 【蓝底专用】直接提取蓝色背景！！！
    // ==============================================
    cv::Mat blue_mask;
    cv::inRange(hsv,
        cv::Scalar(90, 60, 60),    // 蓝色下限
        cv::Scalar(130, 255, 255), // 蓝色上限
        blue_mask);

    // 2. 反向！！！蓝色变黑色，白色赛道变白色
    cv::bitwise_not(blue_mask, this->binary_frame);

    // 3. 轻微降噪，让白线更连续
    cv::GaussianBlur(this->binary_frame, this->binary_frame, cv::Size(3, 3), 0);
    
    if (this->is_mat_valid(this->binary_frame))
    {
        // 返回克隆体，避免外部修改成员变量
        return this->binary_frame.clone();
    }
    return cv::Mat();
}

/********************************************************************************
 * @brief   获取原始图像数据.
 * @param   none.
 * @return  原始图像数据.
 * @example std::vector<uint8_t> raw_frame_data = cam.get_raw_frame_data();
 * @note    none.
 ********************************************************************************/
std::vector<uint8_t> lq_camera::get_raw_frame_data()
{
    // 获取原始图像数据
    std::vector<uint8_t> pixel_vector;
    if (!this->is_mat_valid(this->frame))
    {
        lq_log_error("No volid raw frame data");
        return pixel_vector;
    }
    std::lock_guard<std::mutex> lock(this->mtx);
    // 预分配内存
    pixel_vector.reserve(this->frame.total() * this->frame.channels());
    // 拷贝数据
    pixel_vector.assign(this->frame.data, this->frame.data + this->frame.total() * this->frame.channels());
    return pixel_vector;
}

/********************************************************************************
 * @brief   获取灰度图像数据.
 * @param   none.
 * @return  灰度图像数据.
 * @example std::vector<uint8_t> gray_frame_data = cam.get_gray_frame_data();
 * @note    none.
 ********************************************************************************/
std::vector<uint8_t> lq_camera::get_gray_frame_data()
{
    // 获取灰度图像数据
    std::vector<uint8_t> pixel_vector;
    if (this->get_gray_frame().empty())
    {
        lq_log_error("No volid gray frame data");
        return pixel_vector;
    }
    std::lock_guard<std::mutex> lock(this->mtx);
    // 预分配内存
    pixel_vector.reserve(this->gray_frame.total());
    // 拷贝数据
    pixel_vector.assign(this->gray_frame.data, this->gray_frame.data + this->gray_frame.total());
    return pixel_vector;
}

/********************************************************************************
 * @brief   获取二值图像数据.
 * @param   none.
 * @return  二值图像数据.
 * @example std::vector<uint8_t> binary_frame_data = cam.get_binary_frame_data();
 * @note    none.
 ********************************************************************************/
std::vector<uint8_t> lq_camera::get_binary_frame_data(double _thresh, double _max)
{
    // 获取灰度图像数据
    std::vector<uint8_t> pixel_vector;
    if (this->get_binary_frame(_thresh, _max).empty())
    {
        lq_log_error("No volid binary frame data");
        return pixel_vector;
    }
    std::lock_guard<std::mutex> lock(this->mtx);
    // 预分配内存
    pixel_vector.reserve(this->binary_frame.total());
    // 拷贝数据
    pixel_vector.assign(this->binary_frame.data, this->binary_frame.data + this->binary_frame.total());
    return pixel_vector;
}
#endif

/********************************************************************************
 * @brief   摄像头是否打开.
 * @param   none.
 * @return  打开返回true, 关闭返回false.
 * @example cam.is_opened();
 * @note    none.
 ********************************************************************************/
bool lq_camera::is_opened() const
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->is_open;
}

/********************************************************************************
 * @brief   获取摄像头高度.
 * @param   none.
 * @return  高度.
 * @example uint16_t height = cam.get_height();
 * @note    none.
 ********************************************************************************/
uint16_t lq_camera::get_height() const
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->height;
}

/********************************************************************************
 * @brief   获取摄像头宽度.
 * @param   none.
 * @return  宽度.
 * @example uint16_t width = cam.get_width();
 * @note    none.
 ********************************************************************************/
uint16_t lq_camera::get_width() const
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->width;
}

/********************************************************************************
 * @brief   获取摄像头帧率.
 * @param   none.
 * @return  帧率.
 * @example uint16_t fps = cam.get_fps();
 * @note    none.
 ********************************************************************************/
uint16_t lq_camera::get_fps() const
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->fps;
}

/********************************************************************************
 * @brief   获取摄像头设备ID.
 * @param   none.
 * @return  设备ID.
 * @example int dev_id = cam.get_dev_id();
 * @note    none.
 ********************************************************************************/
int lq_camera::get_dev_id() const
{
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->dev_id;
}
