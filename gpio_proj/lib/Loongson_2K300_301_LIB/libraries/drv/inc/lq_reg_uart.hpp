#ifndef __LQ_REG_UART_HPP
#define __LQ_REG_UART_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include "lq_reg_gpio.hpp"
#include "lq_clock.hpp"
#include "lq_common.hpp"

/****************************************************************************************************
 * @brief   宏参数定义
 ****************************************************************************************************/

#define LS_UART_BASE_ADDR           ( 0x16100000UL )        // UART 寄存器基地址
#define LS_UART_BASE_OFS            ( 0x400UL )             // 串口配置寄存器偏移量

/********************************* UART控制时钟周期 ************************************/

#define LS_UART_CLK_FRE             ( LS_PMON_CLOCK_FREQ )  // UART 控制器时钟周期

/****************************************************************************************************
 * @brief   寄存器结构体定义
 ****************************************************************************************************/

/* 强制1字节对齐，消除编译器填充，保证偏移严格对应 */
#pragma pack(1)
typedef struct ls_uart_reg
{
    union {
        volatile uint8_t DAT;	/*<! 数据传输寄存器 */
        volatile uint8_t DL_L;	/*<! 分频锁存器低 8 位 */
    };
    union {
        volatile uint8_t IER;	/*<! 中断使能寄存器 */
        volatile uint8_t DL_H;	/*<! 分频锁存器高 8 位 */
    };
    union {
        volatile uint8_t IIR;	/*<! 中断源寄存器 */
        volatile uint8_t FCR;	/*<! FIFO 控制寄存器 */
        volatile uint8_t DL_D;	/*<! 分频锁存器小数位 */
    };
    volatile uint8_t LCR;		/*<! 线路控制寄存器 */
    volatile uint8_t MCR;		/*<! MODEM 控制寄存器 */
    volatile uint8_t LSR;		/*<! 线路状态寄存器 */
    volatile uint8_t MSR;		/*<! MODEM 状态寄存器 */
} ls_uart_reg_t;
#pragma pack() // 恢复默认对齐规则

/****************************************************************************************************
 * @brief   枚举值定义
 ****************************************************************************************************/

typedef enum ls_uart_data_bits
{
	LS_UART_DATA5 = 0x00,	        /*<! 5位数据 */
	LS_UART_DATA6 = 0x01,	        /*<! 6位数据 */
	LS_UART_DATA7 = 0x02,	        /*<! 7位数据 */
	LS_UART_DATA8 = 0x03,	        /*<! 8位数据 */
    LS_UART_DATA_INVALID = 0xFF,    /*<! 无效数据位 */
} ls_uart_data_bits_t;

/* 停止位相关枚举类型, 请勿修改 */
typedef enum ls_uart_stop_bits
{
    LS_UART_STOP1 = 0x00,	        /*<! 1 位停止位 */
    LS_UART_STOP2 = 0x04,	        /*<! 2 位停止位 */
    LS_UART_STOP_INVALID = 0xFF,    /*<! 无效停止位 */
} ls_uart_stop_bits_t;

/* 校验位相关枚举类型, 请勿修改 */
typedef enum ls_uart_parity
{
    LS_UART_PARITY_NONE = 0x00,	    /*<! 无校验位 */
    LS_UART_PARITY_ODD  = 0x08,	    /*<! 奇校验位 */
    LS_UART_PARITY_EVEN = 0x18,	    /*<! 偶校验位 */
    LS_UART_PARITY_INVALID = 0xFF,  /*<! 无效校验位 */
} ls_uart_parity_t;

/* 串口端口号, 请勿修改 */
typedef enum uart_port
{
    UART_PORT_0 = 0x00,	/*<! 串口 0 */
    UART_PORT_1,		/*<! 串口 1 */
    UART_PORT_2,		/*<! 串口 2 */
    UART_PORT_3,		/*<! 串口 3 */
    UART_PORT_4,		/*<! 串口 4 */
    UART_PORT_5,		/*<! 串口 5 */
    UART_PORT_6,		/*<! 串口 6 */
    UART_PORT_7,		/*<! 串口 7 */
    UART_PORT_8,		/*<! 串口 8 */
    UART_PORT_9,		/*<! 串口 9 */
    UART_PORT_MAX,		/*<! 串口最大数量 */
	UART_PORT_INVALID	/*<! 无效串口端口号 */
} uart_port_t;

/* 串口可选引脚, 请勿修改 */
typedef enum uart_pin
{
    /* 串口 0 已被终端占用, 非特殊情况禁止使用 */
    // UART0_PIN40 = (UART_PORT_0<<10)|(GPIO_MUX_MAIN<<8)|PIN_40,   /* RX,TX */
    // UART0_PIN92 = (UART_PORT_0<<10)|(GPIO_MUX_ALT2<<8)|PIN_92,

    /* 串口 1 可用引脚 */
    UART1_PIN42 = (UART_PORT_1<<10)|(GPIO_MUX_MAIN<<8)|PIN_42,      /* RX,TX */
    // UART1_PIN94 = (UART_PORT_1<<10)|(GPIO_MUX_ALT2<<8)|PIN_94,   /* 已占用, 未引出 */
    
    /* 串口 2 可用引脚 */
    UART2_PIN44 = (UART_PORT_2<<10)|(GPIO_MUX_MAIN<<8)|PIN_44,      /* TX,RX */
    // UART2_PIN96 = (UART_PORT_2<<10)|(GPIO_MUX_ALT2<<8)|PIN_96,   /* 已占用, 未引出 */
    
    /* 串口 3 可用引脚 */
    UART3_PIN46 = (UART_PORT_3<<10)|(GPIO_MUX_MAIN<<8)|PIN_46,      /* TX,RX */
    // UART3_PIN98 = (UART_PORT_3<<10)|(GPIO_MUX_ALT2<<8)|PIN_98,   /* 已占用, 未引出 */
    
    /* 串口 4 可用引脚 */
    UART4_PIN62 = (UART_PORT_4<<10)|(GPIO_MUX_ALT2<<8)|PIN_62,      /* RX,TX */

    /* 串口 5 可用引脚 */
    UART5_PIN64 = (UART_PORT_5<<10)|(GPIO_MUX_ALT2<<8)|PIN_64,      /* RX,TX */

    /* 串口 6 可用引脚 */
    UART6_PIN60 = (UART_PORT_6<<10)|(GPIO_MUX_ALT2<<8)|PIN_60,      /* TX,RX */
    
    /* 串口 7 已占用, 未引出 */
    // UART7_PIN68 = (UART_PORT_7<<10)|(GPIO_MUX_ALT2<<8)|PIN_68,

    /* 串口 8 已占用, 未引出 */
    // UART8_PIN70 = (UART_PORT_8<<10)|(GPIO_MUX_ALT2<<8)|PIN_70,
    
    /* 串口 9 暂不可用 */
    // UART9_PIN66 = (UART_PORT_9<<10)|(GPIO_MUX_ALT2<<8)|PIN_66,
} uart_pin_t;

/* 接收模式枚举, 请勿修改 */
typedef enum
{
    UART_MODE_BLOCKING = 0,     /*<! 阻塞模式（原始模式） */
    UART_MODE_THREAD   = 1,     /*<! 独立线程模式（推荐，不影响主线程） */
} ls_uart_rx_mode_t;

/* 接收回调函数类型 */
typedef std::function<void(const uint8_t data)> ls_uart_rx_callback_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class ls_uart : public lq_auto_cleanup
{
public:
	// 构造函数
    ls_uart(uart_pin_t            _pin,                             /*<! 串口引脚 */
            uint32_t              _baud    = 115200,                /*<! 默认波特率为 115200 */
			ls_uart_data_bits_t   _data    = LS_UART_DATA8,         /*<! 默认 8 位数据 */
            ls_uart_stop_bits_t   _stop    = LS_UART_STOP1,         /*<! 默认 1 位停止位 */
            ls_uart_parity_t      _parity  = LS_UART_PARITY_NONE,   /*<! 默认无校验位 */
            ls_uart_rx_mode_t     _rx_mode = UART_MODE_THREAD,      /*<! 默认独立线程接收模式 */
            ls_uart_rx_callback_t _cb      = nullptr);              /*<! 接收回调函数, 仅在独立线程接收模式下有效 */
    // 析构函数
	~ls_uart();

public:
	/********************************************************************************
	 * @brief   UART 发送数据.
	 * @param   _buf     : 发送数据缓冲区指针.
	 * @param   _len     : 发送数据长度.
	 * @param   _timeout : 超时时间, 单位: 毫秒.
	 * @return  发送数据长度.
	 * @note    发送数据时, 会阻塞等待, 直到发送完成或超时.
	 ********************************************************************************/
    ssize_t uart_write(const uint8_t *_buf, ssize_t _len, uint32_t _timeout = 100);

	/********************************************************************************
	 * @brief   UART 接收数据.
	 * @param   _buf     : 接收数据缓冲区指针.
	 * @param   _len     : 接收数据长度.
	 * @param   _timeout : 超时时间, 单位: 毫秒.
	 * @return  接收数据长度.
	 * @note    接收数据时, 会阻塞等待, 直到接收完成或超时.
	 ********************************************************************************/
    ssize_t uart_read(uint8_t *_buf, ssize_t _len, uint32_t _timeout = 100);

public:
    /********************************************************************************
     * @brief   设置接收回调函数.
     * @param   _cb : 接收回调函数.
     * @return  none.
     * @note    在线程接收模式下，当接收到数据时会自动调用该回调函数.
     ********************************************************************************/
    void uart_set_rx_callback(ls_uart_rx_callback_t _cb);

    /********************************************************************************
     * @brief   启动接收线程.
     * @param   none.
     * @return  none.
     * @note    内部调用.
     ********************************************************************************/
    void uart_start_rx_thread();

    /********************************************************************************
     * @brief   停止接收线程.
     * @param   none.
     * @return  none.
     * @note    外部可主动调用停止线程.
     ********************************************************************************/
    void uart_stop_rx_thread();

public:
    uart_port_t         get_uart_port() const;  // 获取串口端口号
    ls_uart_data_bits_t get_data_bits() const;  // 获取数据位
    ls_uart_stop_bits_t get_stop_bits() const;  // 获取停止位
    ls_uart_parity_t    get_parity()    const;  // 获取校验位
    uint32_t            get_baudrate()  const;  // 获取波特率

private:
    void cleanup() override;    // 重载 cleanup 函数

public:
    ls_uart(const ls_uart& other) = delete;             // 禁用复制构造函数
    ls_uart(ls_uart&& other)      = delete;             // 禁用移动构造函数
    ls_uart& operator=(const ls_uart& other) = delete;  // 禁用复制赋值运算符
    ls_uart& operator=(ls_uart&& other)      = delete;  // 禁用移动赋值运算符

private:
    gpio_pin_t          rx_pin;     // 接收引脚
    gpio_pin_t          tx_pin;     // 发送引脚
    uart_port_t         port;       // 串口端口号
    gpio_mux_mode_t     mux;        // GPIO 复用模式
    ls_uart_data_bits_t data_bits;  // 数据位
    ls_uart_stop_bits_t stop_bits;  // 停止位
    ls_uart_parity_t    parity;     // 校验位
    uint32_t            baudrate;   // 波特率

    std::mutex  rx_mtx;     // 接收互斥锁
    std::mutex  tx_mtx;     // 发送互斥锁

private:
    ls_uart_rx_mode_t       m_rx_mode;      // 接收模式
    ls_uart_rx_callback_t   m_rx_cb;        // 接收回调函数
    std::thread             m_rx_thread;    // 接收线程
    std::atomic<bool>       m_running;      // 线程运行标志

private:
    void rx_thread_func();  // 接收线程函数

private:
    ls_uart_reg_t *uart_reg;    // UART 寄存器指针
};

/****************************************************************************************************
 * @brief   寄存器位定义
 ****************************************************************************************************/

/*<! 中断使能寄存器 IER */
#define LS_UART_IER_IRxE			BIT(0)	/*<! 接收有效数据中断使能 */
#define LS_UART_IER_ITxE        	BIT(1)	/*<! 传输保存寄存器为空中断使能 */
#define LS_UART_IER_ILE         	BIT(2)	/*<! 接收器线路状态中断使能 */
#define LS_UART_IER_IME         	BIT(3)	/*<! Modem 状态中断使能 */
#define LS_UART_IER_RXDE        	BIT(4)	/*<! 接收状态 DMA 使能 */
#define LS_UART_IER_TXDE        	BIT(5)	/*<! 发送状态 DMA 使能 */
#define LS_UART_IER_ARTSE       	BIT(6)	/*<! RTS 自动流控使能 */
#define LS_UART_IER_ACTSE       	BIT(7)	/*<! CTS 自动流控使能  */

/*<! 中断标识寄存器 IIR */
#define LS_UART_IIR_INTp         	BIT(0)  /*<! 中断表示位 */
#define LS_UART_IIR_II_MASK      	0x0E 	/*<! 中断源表示位掩码 */
#define LS_UART_IIR_II_MODEM_SYS  	0x00  	/*<! 中断类型：Modem 状态 */
#define LS_UART_IIR_II_TX_EMPTY   	0x02  	/*<! 中断类型：传输保存寄存器为空 */
#define LS_UART_IIR_II_RX_TIMEOUT	0x0C  	/*<! 中断类型：接收超时 */
#define LS_UART_IIR_II_RX_RDY      	0x04  	/*<! 中断类型：接收到有效数据 */
#define LS_UART_IIR_II_LINE_STS    	0x06  	/*<! 中断类型：接收线路状态 */

/*<! FIFO 控制寄存器 FCR */
#define LS_UART_FCR_RXSET        	BIT(1)	/*<! 清除接收 FIFO 的内容, 复位其逻辑 */
#define LS_UART_FCR_TXSET        	BIT(2)	/*<! 清除发送 FIFO 的内容, 复位其逻辑 */
#define LS_UART_FCR_TL_MASK      	0xC0  	/*<! 接收 FIFO 提出中断申请的 trigger 值掩码 */
#define LS_UART_FCR_TL_0         	0x00	/*<! 1 字节 */
#define LS_UART_FCR_TL_1         	0x40	/*<! 2 字节 */
#define LS_UART_FCR_TL_2         	0x80	/*<! 3 字节 */
#define LS_UART_FCR_TL_3         	0xC0	/*<! 4 字节 */

/*<! 线路控制寄存器 LCR */
#define LS_UART_LCR_BEC_MSK       	0x03 	/*<! 设定每个字符的位数掩码 */
#define LS_UART_LCR_SB            	BIT(2)  /*<! 定义生成停止位的位数: 0->1 bit, 1->5位字符时为1.5个停止位, 其他长度为2个停止位 */
#define LS_UART_LCR_PE          	BIT(3)  /*<! 奇偶校验位使能 */
#define LS_UART_LCR_EPS            	BIT(4)  /*<! 奇偶校验位选择 */
#define LS_UART_LCR_SPB            	BIT(5)  /*<! 指定奇偶校验位 */
#define LS_UART_LCR_BCB            	BIT(6)  /*<! 打断控制位 */
#define LS_UART_LCR_DLAB            BIT(7)  /*<! 分频锁寄存器访问位 */

/*<! Modem 控制寄存器 MCR */
#define LS_UART_MCR_DTRC            BIT(0)  /*<! DTR 信号控制位 */
#define LS_UART_MCR_RTSC            BIT(1)  /*<! RTS 信号控制位 */
#define LS_UART_MCR_OUT1            BIT(2)  /*<! 在回环模式中连到 RI 输入 */
#define LS_UART_MCR_OUT2            BIT(3)  /*<! 在回环模式中连到 DCD 输入 */
#define LS_UART_MCR_LOOP            BIT(4)  /*<! 回环模式控制位 */

/*<! 线路状态寄存器 LSR */
#define LS_UART_LSR_DR              BIT(0)  /*<! 接收数据有效表示位 */
#define LS_UART_LSR_OE              BIT(1)  /*<! 数据溢出表示位 */
#define LS_UART_LSR_PE              BIT(2)  /*<! 奇偶校验位错误表示位 */
#define LS_UART_LSR_FE              BIT(3)  /*<! 帧错误表示位 */
#define LS_UART_LSR_BI              BIT(4)  /*<! 打断中断表示位 */
#define LS_UART_LSR_TFE             BIT(5)  /*<! 传输 FIFO 位空表示位 */
#define LS_UART_LSR_TE              BIT(6)  /*<! 传输为空表示位 */
#define LS_UART_LSR_ERROR           BIT(7)  /*<! 错误表示位 */

/*<! Modem 状态寄存器 MSR */
#define LS_UART_MSR_DCTS            BIT(0)  /*<! DCTS 指示位 */
#define LS_UART_MSR_DDSR            BIT(1)  /*<! DDSR 指示位 */
#define LS_UART_MSR_TERI            BIT(2)  /*<! RI 边沿检测. RI 状态从低到高变化 */
#define LS_UART_MSR_DDCD            BIT(3)  /*<! DDCD 指示位 */
#define LS_UART_MSR_CCTS            BIT(4)  /*<! CTS 输入值的反, 或者在回环模式中连到 RTS */
#define LS_UART_MSR_CDSR            BIT(5)  /*<! DSR 输入值的反, 或者在回环模式中连到 DTR */
#define LS_UART_MSR_CRI             BIT(6)  /*<! RI 输入值的反, 或者在回环模式中连到 OUT1 */
#define LS_UART_MSR_CDCD            BIT(7)  /*<! DCD 输入值的反, 或者在回环模式中连到 OUT2 */

#endif
