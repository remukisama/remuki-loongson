/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-03-03
@修改内容：新增线程接收模式
@注意事项：使用前需确保CAN接口已正确配置
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "lq_canfd.hpp"
#include "lq_assert.hpp"

// 静态成员初始化
ls_canfd* ls_canfd::s_instance = nullptr;

/********************************************************************************
 * @brief   CAN FD 类的无参构造函数.
 * @param   none.
 * @return  none.
 * @example ls_canfd can.
 * @note    none.
 ********************************************************************************/
ls_canfd::ls_canfd() : m_socket(-1) ,m_rx_mode(CANFD_MODE_THREAD) ,m_rx_cb(nullptr) ,m_initialized(false) ,m_running(false)
{
}

/********************************************************************************
 * @brief   CAN FD 类的带参构造函数.
 * @param   ifname   : CAN FD 接口名称, 如 "can0", "can1".
 * @param   rx_mode  : 接收模式, 参考 ls_canfd_rx_mode_t 枚举.
 * @param   callback : 接收回调函数.
 * @return  none.
 * @example ls_canfd can(CAN1, CANFD_MODE_THREAD, callback).
 * @note    none.
 ********************************************************************************/
ls_canfd::ls_canfd(const std::string &ifname, ls_canfd_rx_mode_t rx_mode, ls_canfd_rx_callback_t _cb)
    : m_socket(-1) ,m_rx_mode(rx_mode) ,m_rx_cb(_cb) ,m_initialized(false) ,m_running(false)
{
    this->canfd_init(ifname, rx_mode, _cb);
}

/********************************************************************************
 * @brief   初始化 CAN FD.
 * @param   ifname   : CAN FD 接口名称, 如 "can0", "can1".
 * @param   rx_mode  : 接收模式, 参考 ls_canfd_rx_mode_t 枚举.
 * @param   callback : 接收回调函数.
 * @return  true:初始化成功 false:初始化失败.
 * @example can.canfd_init(CAN1, CANFD_MODE_THREAD, callback).
 * @note    none.
 ********************************************************************************/
bool ls_canfd::canfd_init(const std::string &ifname, ls_canfd_rx_mode_t rx_mode, ls_canfd_rx_callback_t _cb)
{
    struct sockaddr_can addr;   // CAN FD 地址结构体
    struct ifreq ifr;           // 接口请求结构体
    char cmd[256];              // 命令缓冲区
    // 保存参数
    this->m_ifname  = ifname;
    this->m_rx_mode = rx_mode;
    this->m_rx_cb   = _cb;
    // 先关闭 CAN 接口（如果已打开）
    snprintf(cmd, sizeof(cmd), "ip link set %s down", ifname.c_str());
    system(cmd);
    // 配置并启动 CAN 接口 (CANFD模式: 500kbps 常规 + 2Mbps 数据速率)
    snprintf(cmd, sizeof(cmd), "ip link set %s up type can bitrate 500000 dbitrate 2000000 fd on", ifname.c_str());
    lq_log_info("执行CAN接口配置命令: %s", cmd);
    int ret = system(cmd);
    if (ret != 0) {
        lq_log_error("CANFD 接口配置命令执行失败！");
        return false;
    }
    // 等待接口启动
    usleep(100 * 1000);  // 100ms
    // 创建 socket
    this->m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->m_socket < 0) {
        lq_log_error("CANFD socket 创建失败！");
        return false;
    }
    // 启用 CANFD 模式
    int enable_fd = 1;
    if (setsockopt(this->m_socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
        lq_log_error("启用 CANFD 模式失败！");
        close(this->m_socket);
        this->m_socket = -1;
        return false;
    }
    // 获取接口索引
    strcpy(ifr.ifr_name, ifname.c_str());
    if (ioctl(this->m_socket, SIOCGIFINDEX, &ifr) < 0) {
        lq_log_error("绑定 CAN 接口失败！");
        close(this->m_socket);
        this->m_socket = -1;
        return false;
    }
    // 绑定 CAN 接口 
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(this->m_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        lq_log_error("绑定 CAN 接口失败！");
        close(this->m_socket);
        this->m_socket = -1;
        return false;
    }
    // 根据模式设置接收方式
    if (this->m_rx_mode == CANFD_MODE_ASYNC) {
        if (!this->setup_async_mode()) {    // 异步信号模式
            lq_log_error("设置异步信号接收模式失败！");
            close(this->m_socket);
            this->m_socket = -1;
            return false;
        }
    } else if (this->m_rx_mode == CANFD_MODE_THREAD) {
        this->start_rx_thread();      // 独立线程模式（推荐）
    } else {
        // 阻塞模式设置非阻塞标志（用于poll超时）
        int flags = fcntl(this->m_socket, F_GETFL, 0);
        fcntl(this->m_socket, F_SETFL, flags | O_NONBLOCK);
    }
    // 设置初始化标志
    this->m_initialized = true;
    // 打印成功信息
    const char* mode_str = "阻塞模式";
    if      (this->m_rx_mode == CANFD_MODE_ASYNC)  mode_str = "异步信号模式";
    else if (this->m_rx_mode == CANFD_MODE_THREAD) mode_str = "独立线程模式";
    lq_log_info("CANFD 初始化成功 (%s), 模式: %s\n", this->m_ifname.c_str(), mode_str);
    return true;
}

/********************************************************************************
 * @brief   发送 CAN FD 数据.
 * @param   can_id : CAN ID.
 * @param   data   : 数据指针.
 * @param   len    : 数据长度（最大64字节）.
 * @return  发送字节数，-1表示失败.
 * @example can.canfd_write_data(0x123, data, 8);
 * @note    none.
 ********************************************************************************/
int ls_canfd::canfd_write_data(uint32_t _can_id, const uint8_t *_data, uint8_t _len)
{
    if (!this->m_initialized || this->m_socket < 0) {
        lq_log_error("CANFD 未初始化或socket无效");
        return -1;
    }
    if (_len > CANFD_MAX_DATA_LEN) {
        lq_log_warn("CANFD 发送数据过长");
        _len = CANFD_MAX_DATA_LEN;
    }
    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = _can_id;
    frame.len    = _len;
    memcpy(frame.data, _data, _len);
    return write(this->m_socket, &frame, sizeof(frame));
}

/********************************************************************************
 * @brief   发送 CAN FD 数据帧.
 * @param   _frame : CAN FD 数据帧.
 * @return  发送字节数，-1表示失败.
 * @example can.canfd_write_frame(frame);
 * @note    none.
 ********************************************************************************/
int ls_canfd::canfd_write_frame(const ls_canfd_frame_t &_frame)
{
    return canfd_write_data(_frame.can_id, _frame.data, _frame.len);
}

/********************************************************************************
 * @brief   阻塞接收 CAN FD 数据.
 * @param   _frame      : 接收数据帧结构体.
 * @param   _timeout_ms : 超时时间(毫秒)，-1表示无限等待
 * @return  接收字节数，-1表示失败，0表示超时.
 * @example can.canfd_read_frame(frame, 1000);
 * @note    适用于阻塞模式.
 ********************************************************************************/
int ls_canfd::canfd_read_frame(ls_canfd_frame_t &_frame, int _timeout_ms)
{
    if (!this->m_initialized || this->m_socket < 0) {
        lq_log_error("CANFD 未初始化或socket无效");
        return -1;
    }
    struct pollfd pfd;
    pfd.fd = this->m_socket;
    pfd.events = POLLIN;
    // 等待数据
    int ret = poll(&pfd, 1, _timeout_ms);
    if (ret < 0) {
        lq_log_error("CANFD poll 失败");
        return -1;
    }
    if (ret == 0) {
        lq_log_warn("CANFD read 接收超时");
        return 0;  // 超时
    }
    // 读取数据
    struct canfd_frame can_frame;
    int nbytes = read(this->m_socket, &can_frame, sizeof(can_frame));
    if (nbytes <= 0) {
        lq_log_error("CANFD read 读取数据失败");
        return -1;
    }
    // 填充帧结构体
    _frame.can_id = can_frame.can_id;
    _frame.len = can_frame.len;
    memcpy(_frame.data, can_frame.data, can_frame.len);

    return nbytes;
}

/********************************************************************************
 * @brief   设置接受回调函数.
 * @param   _cb : 接收数据帧结构体.
 * @return  接收字节数，-1表示失败，0表示超时.
 * @example can.set_rx_callback([](const ls_canfd_frame_t &frame){...});
 * @note    异步模式或独立线程模式下使用.
 ********************************************************************************/
 void ls_canfd::set_rx_callback(ls_canfd_rx_callback_t _cb)
 {
     this->m_rx_cb = _cb;
 }

/********************************************************************************
 * @brief   设置异步信号接收模式.
 * @param   none.
 * @return  true:成功 false:失败.
 * @example 内部调用.
 * @note    会中断主线程的 sleep.
 ********************************************************************************/
bool ls_canfd::setup_async_mode()
{
    // 设置信号处理函数
    signal(SIGIO, this->signal_handler);
    // 设置socket所有者
    if (fcntl(m_socket, F_SETOWN, getpid()) < 0) {
        lq_log_error("CANFD fcntl F_SETOWN 设置失败");
        return false;
    }
    // 启用异步I/O
    int flags = fcntl(m_socket, F_GETFL);
    if (fcntl(m_socket, F_SETFL, flags | O_ASYNC | O_NONBLOCK) < 0) {
        lq_log_error("CANFD fcntl F_SETFL 启用异步I/O失败");
        return false;
    }
    // 保存实例指针
    this->s_instance = this;

    return true;
}

/********************************************************************************
 * @brief   启动接收线程.
 * @param   none.
 * @return  none.
 * @example 内部调用.
 * @note    独立线程接收，不影响主线程.
 ********************************************************************************/
void ls_canfd::start_rx_thread()
{
    this->m_running = true;
    this->m_rx_thread = std::thread(&ls_canfd::rx_thread_func, this);
}

/********************************************************************************
 * @brief   接收线程函数.
 * @param   none.
 * @return  none.
 * @example 内部调用.
 * @note    独立线程接收，可安全调用回调.
 ********************************************************************************/
void ls_canfd::rx_thread_func()
{
    struct pollfd pfd;
    pfd.fd = this->m_socket;
    pfd.events = POLLIN;

    while (this->m_running) {
        // 等待数据，100ms超时检查一次运行状态
        int ret = poll(&pfd, 1, 100);
        if (!this->m_running) {
            break;
        }
        if (ret > 0 && (pfd.revents & POLLIN)) {
            struct canfd_frame frame;
            int nbytes = read(m_socket, &frame, sizeof(frame));   
            if (nbytes > 0 && m_rx_cb != nullptr) {
                ls_canfd_frame_t rx_frame;
                rx_frame.can_id = frame.can_id;
                rx_frame.len = frame.len;
                memcpy(rx_frame.data, frame.data, frame.len);
                // 在独立线程中调用回调，可安全使用printf
                this->m_rx_cb(rx_frame);
            }
        }
    }
}

/********************************************************************************
 * @brief   SIGIO信号处理函数.
 * @param   _signo : 信号编号.
 * @return  none.
 * @example 内部调用.
 * @note    有数据到来时自动触发.
 * @note    信号处理函数中不能使用printf等非async-signal-safe函数.
 ********************************************************************************/
void ls_canfd::signal_handler(int _signo)
{
    if (s_instance == nullptr) {
        lq_log_error("CANFD SIGIO单例指针为空");
        return;
    }
    struct canfd_frame frame;
    int nbytes;
    // 读取所有可用数据
    while (1) {
        nbytes = read(s_instance->m_socket, &frame, sizeof(frame));
        if (nbytes <= 0) {
            break;
        }
        // 如果有回调函数，构造帧并调用
        if (s_instance->m_rx_cb != nullptr) {
            // 构造帧结构体
            ls_canfd_frame_t rx_frame;
            rx_frame.can_id = frame.can_id;
            rx_frame.len = frame.len;
            memcpy(rx_frame.data, frame.data, frame.len);
            // 注意：回调函数中不要使用printf，会导致阻塞！
            s_instance->m_rx_cb(rx_frame);
        }
    }
}

/********************************************************************************
 * @brief   CNAFD类析构函数.
 * @param   none.
 * @return  none.
 * @note    创建对象生命周期结束后系统自动调用.
 ********************************************************************************/
ls_canfd::~ls_canfd()
{
    // 停止接收线程
    this->m_running = false;
    if (this->m_rx_thread.joinable()) {
        this->m_rx_thread.join();
    }
    if (this->m_socket >= 0) {
        close(this->m_socket);
        this->m_socket = -1;
    }
    if (this->s_instance == this) {
        this->s_instance = nullptr;
    }
}
