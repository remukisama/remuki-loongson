#include "lq_base_class.hpp"
#include <algorithm>
#include <cstdio>

/********************************************************************************
 * @brief   获取全局对象列表.
 * @param   none.
 * @return  全局对象列表引用.
 * @note    该函数返回的引用是静态的, 所以在程序结束时会自动清理所有对象.
 ********************************************************************************/
std::vector<lq_auto_cleanup*>& lq_auto_cleanup::get_global_obj_list()
{
    static std::vector<lq_auto_cleanup*> global_obj_list;
    return global_obj_list;
}

/********************************************************************************
 * @brief   构造函数, 将对象添加到全局对象列表中.
 * @param   none.
 * @return  none.
 * @note    该函数会在继承该类的对象创建时自动调用, 将对象添加到全局对象列表中.
 ********************************************************************************/
lq_auto_cleanup::lq_auto_cleanup()
{
    get_global_obj_list().push_back(this);
}

/********************************************************************************
 * @brief   析构函数, 从全局对象列表中移除对象.
 * @param   none.
 * @return  none.
 * @note    该函数会在继承该类的对象销毁时自动调用, 从全局对象列表中移除对象.
 ********************************************************************************/
lq_auto_cleanup::~lq_auto_cleanup()
{
    auto& list = get_global_obj_list();
    list.erase(std::remove(list.begin(), list.end(), this), list.end());
}

/********************************************************************************
 * @brief   清理所有对象的硬件配置.
 * @param   none.
 * @return  none.
 * @note    该函数会在程序结束时自动调用, 清理所有对象的硬件配置.
 ********************************************************************************/
void lq_auto_cleanup::cleanup_all()
{
    auto& list = get_global_obj_list();
    for (auto obj : list)
    {
        obj->cleanup();
    }
    list.clear();
}
