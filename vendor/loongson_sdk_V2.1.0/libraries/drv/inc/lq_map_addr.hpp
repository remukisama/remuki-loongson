#ifndef __LQ_MAP_ADDR_HPP
#define __LQ_MAP_ADDR_HPP

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <assert.h>
#include <stdexcept>

#include "lq_common.hpp"

/****************************************************************************************************
 * @brief   宏函数定义
 ****************************************************************************************************/

#define ls_readb(addr)          (*(volatile uint8_t*)(addr))
#define ls_writeb(addr, val)    (*(volatile uint8_t*)(addr) = (val))

#define ls_readl(addr)          (*(ls_reg32_addr_t)(addr))         // 读寄存器值
#define ls_writel(addr, val)    (*(ls_reg32_addr_t)(addr) = (val)) // 写寄存器值

// /****************************************************************************************************
//  * @brief   类定义
//  ****************************************************************************************************/

/* 命名空间, 防止命名冲突 */
namespace LQ
{
    /********************************************************************************
     * @class   memory_mapper
     * @brief   内存映射器
     * @example 示例一：C风格独立函数
     *          uint32_t phy_addr = 0x12345678  // 替换为实际物理地址
     *          void *mapped_addr = LQ::lq_addr_mmap(phy_addr);
     *          std::cout << "mapped address (C style):" << mapped_addr << std::endl;
     * 
     * @example 示例二：类静态方法
     *          uint32_t phy_addr = 0x12345678  // 替换为实际物理地址
     *          void *mapped_adddr = LQ::memory_mapper::mmap_addr(phy_addr);
     *          std::cout << "mapped address (static method):" << mapped_addr << std::endl;
     * 
     * @example 示例三：：类实例方法
     *          uint32_t phy_addr = 0x12345678  // 替换为实际物理地址
     *          LQ::memory_mapper mapper;
     *          void *mapped_addr = mapper.map(phy_addr);
     *          std::cout << "mapped address (instance method):" << mapped_addr << std::endl;
     * 
     * @example 释放映射
     *          示例一：LQ::lq_addr_munmap(mapped_addr);
     *          示例二：LQ::memory_mapper::unmap_addr(mapped_addr);
     *          示例三：mapper.unmap_addr(mapped_addr);
     ********************************************************************************/
    class memory_mapper
    {
    public:
        // 静态方法 - 独立调用入口
        static ls_reg32_addr_t mmap_addr(ls_reg_base_t addr);
        // 静态方法 - 内存释放
        static void unmap_addr(ls_reg32_addr_t map_addr);

        // 实例方法 - 类对象调用入口
        ls_reg32_addr_t map(ls_reg_base_t addr);
    
    private:
        // 核心映射逻辑
        static ls_reg32_addr_t map_physical_address(ls_reg_base_t addr);

        // 页大小
        static const uint32_t PAGE_SIZE = 0x10000;
    };

    // 独立内联函数：物理地址映射虚拟地址
    inline ls_reg32_addr_t ls_addr_mmap(ls_reg_base_t addr)
    {
        return memory_mapper::mmap_addr(addr);
    }

    // 独立内联函数：释放映射的虚拟地址
    inline void ls_addr_munmap(ls_reg32_addr_t addr)
    {
        memory_mapper::unmap_addr(addr);
    }
};

#endif
