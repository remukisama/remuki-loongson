#ifndef __LQ_MODULE_LOAD_HPP
#define __LQ_MODULE_LOAD_HPP

#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
#include <unordered_set>
#include <memory>
#include <array>
#include <algorithm>

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class lq_module_load {
public:
    lq_module_load();                                   // 默认构造函数
    lq_module_load(std::string _module);                // 加载单个模块的构造函数
    lq_module_load(std::vector<std::string> _modules);  // 加载多个模块的构造函数
    ~lq_module_load();                                  // 析构函数

public:
    bool        set_load_module(const std::string _module_name);    // 加载驱动模块
    bool        set_unload_module(const std::string _module_name);  // 卸载驱动模块

    void        set_unload_all_module();                            // 卸载所有模块

    bool        is_module_load(const std::string& _module);         // 检测模块是否已加载

private:
    std::string get_module_name(const std::string& _input);         // 提纯模块名
    std::string execute_shell_commands(const std::string& _cmd);    // 执行终端 shell 指令并返回输出结果

private:
    std::vector<std::string>        load_;                  // 加载模块列表
    std::string                     module_path = "/home/"; // 模块搜索路径

};
 
#endif
