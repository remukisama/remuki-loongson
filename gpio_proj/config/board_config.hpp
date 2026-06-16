/**
 * @file board_config.hpp
 * @brief 板级配置分发器
 *
 * 当前仅支持 2K301 龙丘核心板。
 * 新增板子：
 *   1. 创建 config/board_xxx.hpp
 *   2. 在下方 #elif 中添加 #elif defined(BOARD_XXX)
 *   3. CMakeLists.txt 中添加板级选项
 */

#ifndef GPIO_PROJ_BOARD_CONFIG_HPP
#define GPIO_PROJ_BOARD_CONFIG_HPP

#if defined(BOARD_2K301)
    #include "board_2k301.hpp"
#else
    #error "未定义板级宏！请在 CMake 中指定 -DBOARD=2K301"
#endif

#endif /* GPIO_PROJ_BOARD_CONFIG_HPP */
