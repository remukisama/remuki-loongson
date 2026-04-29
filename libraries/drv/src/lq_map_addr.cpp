#include "lq_map_addr.hpp"

namespace LQ
{
    /********************************************************************************
     * @brief   将物理地址映射为虚拟地址, 核心映射逻辑(私有静态方法).
     * @param   addr : 物理地址.
     * @return  成功返回虚拟地址.
     * @example void *addr = memory_mapper::map_physical_address(LS_GPIO_BASE_ADDR);
     * @note    该函数会映射一个页大小的内存区域, 并返回所映射的虚拟地址值.
     * @note    请在使用后调用 memory_mapper::unmap_addr() 释放映射.
     ********************************************************************************/
    ls_reg32_addr_t memory_mapper::map_physical_address(ls_reg_base_t addr)
    {
        int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd < 0)
        {
            assert("open /dev/mem error");
            exit(EXIT_FAILURE);
        }
        void *_addr = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, addr & ~(PAGE_SIZE - 1));
        close(mem_fd);
        if (_addr == MAP_FAILED)
        {
            assert("mmap error");
            exit(EXIT_FAILURE);
        }
        return (ls_reg32_addr_t)((ls_reg_base_t)_addr + (addr & (PAGE_SIZE - 1)));
    }

    /********************************************************************************
     * @brief   将物理地址映射为虚拟地址, 实现静态映射方法.
     * @param   addr : 物理地址.
     * @return  成功返回虚拟地址.
     * @example void *addr = memory_mapper::mmap_addr(LS_GPIO_BASE_ADDR);
     * @note    该函数会映射一个页大小的内存区域, 并返回所映射的虚拟地址值.
     * @note    请在使用后调用 memory_mapper::unmap_addr() 释放映射.
     ********************************************************************************/
    ls_reg32_addr_t memory_mapper::mmap_addr(ls_reg_base_t addr)
    {
        return map_physical_address(addr);
    }

    /********************************************************************************
     * @brief   释放映射的虚拟地址, 实现静态释放方法.
     * @param   addr : 虚拟地址.
     * @return  none.
     * @example memory_mapper::unmap_addr(mapped_addr);
     * @note    该函数会释放映射的虚拟地址, 并调用 munmap() 系统调用.
     ********************************************************************************/
    void memory_mapper::unmap_addr(ls_reg32_addr_t addr)
    {
        if (addr == nullptr || addr == MAP_FAILED)
        {
            return;
        }
        // 计算页对齐的基地址用于释放
        ls_reg_base_t base_addr = reinterpret_cast<ls_reg_base_t>(addr);
        base_addr &= ~(PAGE_SIZE - 1);
        
        if (munmap(reinterpret_cast<void*>(base_addr), PAGE_SIZE) == -1)
        {
            assert("munmap failed");
            return;
        }
    }

    /********************************************************************************
     * @brief   将物理地址映射为虚拟地址, 实现实例映射方法.
     * @param   addr : 物理地址.
     * @return  成功返回虚拟地址.
     * @example void *addr = mapper.map(LS_GPIO_BASE_ADDR);
     * @note    该函数会映射一个页大小的内存区域, 并返回所映射的虚拟地址值.
     * @note    请在使用后调用 memory_mapper::unmap_addr() 释放映射.
     ********************************************************************************/
    ls_reg32_addr_t memory_mapper::map(ls_reg_base_t addr)
    {
        return this->map_physical_address(addr);
    }
};
