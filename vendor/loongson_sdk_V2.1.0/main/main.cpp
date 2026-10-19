/********************************************************************************
 * @file            main.cpp
 * @brief           本文件是 LQ_2K300_301_LIB 软件开源库文件的一部分
 * @copyright       版权所有 (C) 2025-2026 北京龙邱科技有限公司
 * @website         http://www.lqist.cn
 * @taobao          https://longqiu.taobao.com
 *
 * @description     龙邱科技 LS2K300/301 核心板驱动开源库声明
 *
 * 本文件遵循 GPL-3.0 开源协议发布，旨在为龙芯 2K300/301 平台提供快速上手开发基于龙芯 2K300/301 平台的应用程序的参考实现.
 * 商业用途(包括单位使用)需提前联系作者获取授权
 *
 * GPL-3.0 许可证声明摘要:
 * 1. 允许自由使用、修改、分发本软件
 * 2. 分发修改后的版本时，必须以相同许可证发布
 * 3. 必须保留原始版权声明和许可证信息
 * 4. 不提供任何担保，使用风险自负
 * 5. 完整协议文本请参见项目根目录 LICENSE 文件
 *
 * @author          龙邱科技-012
 * @email           chiusir@163.com
 * @version         V2.1.0
 * @update          2026-04-21
 *
 * @note            使用本开源库前, 请确认板卡型号与 PMON 版本是否匹配.
 *                  龙邱 2K301 或已根据群文件升级系统的2K300可直接使用.
 *                  旧版本 2K300 需要修改时钟频率, 请到 lq_clock.hpp 文件修改 CONFIG_USE_PMON.
 *
 * @note            本库已重载 CTRL+C 信号, 按下 CTRL+C 时，会设置 ls_system_running 为 false，
 *                  从而退出所有正在运行的 demo 例程.
 * @note            如果用户在主程序写了自定义的循环，需要在循环中判断 ls_system_running.load() 是否为 true，
 *                  否则会导致程序无法退出.
 ********************************************************************************/

#include "main.hpp"

int main()
{
    while (ls_system_running.load())
    {
        usleep(100 * 1000);
    }

    return 0;
}
