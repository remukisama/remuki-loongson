#ifndef __LQ_CLOCK_HPP
#define __LQ_CLOCK_HPP

/********************************************************************************
 * @brief   这个参数是用来切换新旧PMON不同导致的时钟频率不一样的问题的.
 * @note    旧PMON的时钟频率是100MHz，而新PMON的时钟频率是160MHz，所以需要这个参数来切换
 * @note    当这个参数的值为1时，表示使用新PMON，时钟频率为160MHz.
 * @note    当这个参数的值为0时，表示使用旧PMON，时钟频率为100MHz.
 * @note    2K0301或是更新过群里系统的2K0300, PMON时钟频率均为160MHz.
 ********************************************************************************/
#define CONFIG_USE_PMON         ( 1 )

#if (CONFIG_USE_PMON == 1)
    #define LS_PMON_CLOCK_FREQ  ( 160000000L )
#else
    #define LS_PMON_CLOCK_FREQ  ( 100000000L )
#endif

#endif
