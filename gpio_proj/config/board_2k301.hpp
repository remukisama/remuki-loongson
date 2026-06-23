/**
 * @file board_2k301.hpp
 * @brief 龙邱 LS2K0301 双龙派母板 全量引脚配置
 *
 * 引脚来源：龙邱官方拓展版 GPIO 资源分配表
 *
 * 看门狗保护范围（心跳超时写安全值）：
 *   - 4路电机 PWM（ATIM TIM1_CH1~CH4）
 *   - 4路电机方向 GPIO
 *   - 1路舵机 PWM（GTIM TIM2_CH2）
 *   - 蜂鸣器 GPIO
 *
 * 不保护的引脚（不会导致失控）：
 *   - 编码器（只读，无输出风险）
 *   - 按键、拨码开关（只读输入）
 *   - 屏幕、串口、I2C（无运动风险）
 */

#ifndef GPIO_PROJ_BOARD_2K301_HPP
#define GPIO_PROJ_BOARD_2K301_HPP

// ==================== PWM 控制器类型 ====================
#define PWM_TYPE_ATIM                       // 电机使用 ATIM（高级定时器）
#define PWM_CLK_FREQ    160000000UL         // 160MHz（新PMON）
#undef  PWM_DUTY_MAX
#define PWM_DUTY_MAX    10000               // ATIM/GTIM 占空比量程 0~10000

// ==================== 电机 PWM（ATIM TIM1_CH1~CH4）====================
#define PIN_MOTOR_PWM_1         ATIM_PWM0_PIN81     // GPIO81, TIM1_CH1, MAIN复用
#define PIN_MOTOR_PWM_2         ATIM_PWM1_PIN82     // GPIO82, TIM1_CH2, MAIN复用
#define PIN_MOTOR_PWM_3         ATIM_PWM2_PIN83     // GPIO83, TIM1_CH3, MAIN复用
#define PIN_MOTOR_PWM_4         ATIM_PWM3_PIN76     // GPIO76, TIM1_CH4, ALT1复用
#define PIN_MOTOR_PWM_FREQ      25000               // 25kHz
#define PIN_MOTOR_PWM_POLARITY  ATIM_PWM_POL_NORMAL // 当前双路驱动板实测使用正常极性

// ==================== 电机方向 GPIO ====================
#define PIN_MOTOR_DIR_1         21          // GPIO21（LCD_D17）— 电机1方向
#define PIN_MOTOR_DIR_2         22          // GPIO22（LCD_D18）— 电机2方向
#define PIN_MOTOR_DIR_3         23          // GPIO23（LCD_D19）— 电机3方向
#define PIN_MOTOR_DIR_4         24          // GPIO24（LCD_D20）— 电机4方向

// ==================== 编码器（4路）====================
#define PIN_ENCODER_1           ENC_PWM0_PIN64      // GPIO64, CH0（脉冲）
#define PIN_ENCODER_1_DIR       72                  // GPIO72（方向）
#define PIN_ENCODER_2           ENC_PWM1_PIN65      // GPIO65, CH1（脉冲）
#define PIN_ENCODER_2_DIR       73                  // GPIO73（方向）
#define PIN_ENCODER_3           ENC_PWM2_PIN66      // GPIO66, CH2（脉冲）
#define PIN_ENCODER_3_DIR       74                  // GPIO74（方向）
#define PIN_ENCODER_4           ENC_PWM3_PIN67      // GPIO67, CH3（脉冲）
#define PIN_ENCODER_4_DIR       75                  // GPIO75（方向）

// ==================== 舵机 PWM（GTIM TIM2_CH2）====================
#define PIN_SERVO_PWM           GTIM_PWM1_PIN88     // GPIO88, TIM2_CH2, MAIN复用
#define PIN_SERVO_PWM_FREQ      50                  // 50Hz（标准舵机频率）
#define PIN_SERVO_PWM_POLARITY  GTIM_PWM_POL_INV    // 反极性

// ==================== 蜂鸣器 ====================
#define PIN_BUZZER              61          // GPIO61

// ==================== 按键（3路，仅供参考）====================
#define PIN_KEY_0               44          // GPIO44（UART2_TX）
#define PIN_KEY_1               45          // GPIO45（UART2_RX）
#define PIN_KEY_2               80          // GPIO80（I2S_DO）

// ==================== 拨码开关（2路，仅供参考）====================
#define PIN_DIPSW_1             17          // GPIO17（LCD_D13）
#define PIN_DIPSW_2             20          // GPIO20（LCD_D16）

// ==================== 屏幕 SPI 接口（仅供参考）====================
#define PIN_SCREEN_DC           48          // GPIO48（数据/命令）
#define PIN_SCREEN_RST          49          // GPIO49（复位）
#define PIN_SCREEN_SCK          60          // GPIO60（SPI时钟）
#define PIN_SCREEN_SDI          62          // GPIO62（SPI数据）
#define PIN_SCREEN_CS           63          // GPIO63（SPI片选）

// ==================== I2C 接口（仅供参考）====================
#define PIN_I2C_SCL             50          // GPIO50（I2C1_SCL）
#define PIN_I2C_SDA             51          // GPIO51（I2C1_SDA）

// ==================== 安全默认值 ====================
// 当前双路驱动板正常极性、低有效：保持通道使能且原始 duty=10000 时电机停止
// GTIM 反极性：duty=0 → 输出HIGH → 舵机中位（或停止）
#define SAFE_MOTOR_DUTY         10000       // 电机停止（100%高电平）
#define SAFE_MOTOR_DIR          0           // 方向 LOW（标定安全方向）
#define SAFE_SERVO_DUTY         0           // 舵机停止
#define SAFE_BUZZER             0           // 静音

// ==================== 兼容旧名（小车工程过渡期）====================
// 小车工程如果用 PIN_MOTOR_PWM_LEFT/RIGHT，自动映射到 1/2
#define PIN_MOTOR_PWM_LEFT      PIN_MOTOR_PWM_1
#define PIN_MOTOR_PWM_RIGHT     PIN_MOTOR_PWM_2
#define PIN_MOTOR_DIR_LEFT      PIN_MOTOR_DIR_1
#define PIN_MOTOR_DIR_RIGHT     PIN_MOTOR_DIR_2
#define PIN_MOTOR_PWM_FREQ_     PIN_MOTOR_PWM_FREQ
#define PIN_MOTOR_PWM_POLARITY_ PIN_MOTOR_PWM_POLARITY
#define PIN_ENCODER_LEFT        PIN_ENCODER_1
#define PIN_ENCODER_LEFT_DIR    PIN_ENCODER_1_DIR
#define PIN_ENCODER_RIGHT       PIN_ENCODER_2
#define PIN_ENCODER_RIGHT_DIR   PIN_ENCODER_2_DIR

#endif /* GPIO_PROJ_BOARD_2K301_HPP */
