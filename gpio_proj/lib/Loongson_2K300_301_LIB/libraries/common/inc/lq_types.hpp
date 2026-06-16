#ifndef __TYPES_H
#define __TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <bits/types.h>

// 存放通用结构体、自定义类型等

typedef uint32_t                ls_reg32_t;         // 32 位寄存器值类型
typedef volatile ls_reg32_t*    ls_reg32_addr_t;    // 32 位寄存器地址/指针类型

typedef uint64_t                ls_reg64_t;         // 64 位寄存器值类型
typedef volatile ls_reg64_t*    ls_reg64_addr_t;    // 64 位寄存器地址/指针类型

typedef uint32_t                ls_reg_off_t;       // 寄存器偏移量类型
typedef uintptr_t               ls_reg_base_t;      // 通用寄存器基地址类型

#define SET     1
#define RESET   0

#define HIGH    1
#define LOW     0

/********************************************************************************
 * @brief   通用寄存器地址计算宏
 * @param   base_ptr : 寄存器基地址.
 * @param   byte_off : 字节偏移量.
 * @return  目标寄存器指针.
 ********************************************************************************/
#define ls_reg_addr_calc(base_ptr, byte_off) \
    ((decltype(base_ptr))((ls_reg_base_t)(base_ptr) + (ls_reg_off_t)(byte_off)))

// 位操作
#define BIT0        ( 0x01 )
#define BIT1        ( 0x02 )
#define BIT2        ( 0x04 )
#define BIT3        ( 0x08 )
#define BIT4        ( 0x10 )
#define BIT5        ( 0x20 )
#define BIT6        ( 0x40 )
#define BIT7        ( 0x80 )
#define BIT(n)      ( 1 << (n) )

// 无参无返回值的回调函数类型
typedef void (*lq_empty_cb_t)(void);

#ifdef __cplusplus
}
#endif

#endif
