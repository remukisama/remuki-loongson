#ifndef __BASE_CLASS_H
#define __BASE_CLASS_H

#include <vector>

// 通用基类


class lq_auto_cleanup
{
private:
    // 全局对象列表(所有创建的对象都会自动存在这里)
    static std::vector<lq_auto_cleanup*>& get_global_obj_list();

public:
    // 主动清理所有对象(可随时调用)
    static void cleanup_all();

public:
    lq_auto_cleanup();          // 构造函数
    virtual ~lq_auto_cleanup(); // 析构函数

public:
    /********************************************************************************
     * @brief   清理函数, 如果继承该类就必须实现该函数, 也是唯一一个需要实现的函数
     * @note    可将需要清理的资源在该函数中释放
     ********************************************************************************/
    virtual void cleanup() = 0;

public:
    lq_auto_cleanup(const lq_auto_cleanup&) = delete;               // 禁用拷贝构造函数
    lq_auto_cleanup(lq_auto_cleanup&&)      = delete;               // 禁用移动构造函数
    lq_auto_cleanup& operator=(const lq_auto_cleanup&) = delete;    // 禁用拷贝赋值运算符
    lq_auto_cleanup& operator=(lq_auto_cleanup&&)      = delete;    // 禁用移动赋值运算符

};

#endif
