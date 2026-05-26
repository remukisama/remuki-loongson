#include "main.hpp"
#include "car_config.hpp"
#include <cstdio>

// =============================================================================
// @file    34_lq_left_ring_exit_recommend_demo.cpp
// @brief   左环岛“进去了但在环岛里面兜圈”专项 Demo
//
// 使用场景：
//   T08 日志已经能看到：
//     RoadChange Normol -> LeftCirque
//     SceneChange Normal -> LeftRing
//     ring=1/5 -> 1/6 -> 1/7/1/8
//   说明识别和进环已经生效；如果实车仍在环内持续同向兜圈，问题重点不再是
//   “继续放宽左环识别/继续增大进环偏置”，而是“flag>=7 后出环释放不够”。
//
// 推荐做法已经写进 car_config.hpp / my_motor.cpp：
//   1) flag=5/6：允许强制保持左转，避免中心线回中后车在环内走直。
//   2) flag>=7：释放强制左转，改为轻微反向/出环转向，避免继续绕圈。
//   3) 环内差速限幅略收小，避免同向角速度长期贴顶。
//   4) 不改速度环 PID；此类问题不是速度 P/I/D 导致。
//
// 本 Demo 做两件事：
//   A. 启动时打印当前编译进去的推荐参数和操作步骤，防止跑错旧程序。
//   B. 默认直接进入 T08 自动低速元素调试，继续输出 Road/Scene/ring/Tgt/GErr 等日志，
//      并在 TFT18 上实时显示补线、中心线、P1/P2 和 ring 状态。
//      若要先手推看图，把 kRunHandPushFirst 改为 true。
// =============================================================================

namespace left_ring_exit_recommend_demo {

static constexpr bool kRunHandPushFirst = false;  // true=先跑T08H手推图传；false=直接自动低速T08

static void print_recommendation_card(void) {
    printf("\n========== DEMO 34 left_ring_exit_recommend 左环出环防兜圈专项 ==========" "\n");
    printf("[现象定位] 你这次日志已经进入左环岛：Road=LeftCirque Scene=LeftRing，并推进到 ring=1/7/1/8。\n");
    printf("[结论] 这不是漏检；不要再继续放宽左环识别，也不要继续增大进环偏置，否则更容易在环里绕圈。\n");
    printf("[本轮推荐] flag=5/6 保持最小进环左转；flag>=7 释放强制左转并给轻微反向出环。\n");
    printf("[不要改] 不要改速度环 PID，不要用环岛问题反推普通二值化/普通弯PID一起改。\n\n");
    printf("[TFT18显示] 屏幕会显示实时二值图、原始/补线边界、最终中心线、P1/P2水平线和 ring 状态；不用额外打开旧的 TFT Demo。\n\n");

    printf("[当前编译参数]\n");
    printf("  kRingEntryCenterBias=%d        // 进环偏置；已能进环后不要再加大\n", car_config::kRingEntryCenterBias);
    printf("  kRingInnerCenterBias=%d        // 环内补线偏置\n", car_config::kRingInnerCenterBias);
    printf("  kRingForcedSteerError=%.1f     // flag=5/6 最小转向，防止环内走直\n", car_config::kRingForcedSteerError);
    printf("  kRingForcedSteerReleaseFlag=%d // 到该 flag 后释放强制同向转向\n", car_config::kRingForcedSteerReleaseFlag);
    printf("  kRingExitSteerError=%.1f       // 左环 flag>=7 轻微反向出环；右环自动取反\n", car_config::kRingExitSteerError);
    printf("  kRingEntryGyroLimitDps=%.1f    // 入环角速度限幅\n", car_config::kRingEntryGyroLimitDps);
    printf("  kRingSceneGyroLimitDps=%.1f    // 环岛场景角速度限幅\n", car_config::kRingSceneGyroLimitDps);
    printf("  kRingEntryMaxDiff=%.1f         // 入环差速限幅\n", car_config::kRingEntryMaxDiff);
    printf("  kRingMaxDiff=%.1f              // 环内差速限幅；绕圈时优先略收小\n\n", car_config::kRingMaxDiff);

    printf("[操作方法]\n");
    printf("  1. 先确认普通循迹稳定，再把车放在左环岛前 30~50cm。\n");
    printf("  2. 运行本 Demo，低速进入左环岛；人在旁边准备随时拿起车。\n");
    printf("  3. 重点观察日志：ring=1/5/6 时允许 Tgt 正向较大；到 ring=1/7/8 后 Tgt 应该开始释放/不再长期同向贴顶。\n");
    printf("  4. 同时看 TFT18：蓝色中心线在 1/5/6 应切向环心，在 1/7/8 应逐步指向出口，不应继续指向环内。\n");
    printf("  5. 如果仍然兜圈，把 [T08_EVENT]、ring=1/7/8 附近的 [T08] 和 [steer] 发回来。\n\n");

    printf("[根据结果下一步]\n");
    printf("  - 如果 ring=1/5 都没有：才回去看识别阈值。\n");
    printf("  - 如果 ring=1/5/6 不进环：微调 kRingEntryCenterBias / 入环差速。\n");
    printf("  - 如果 ring=1/7/8 仍绕圈：优先让 kRingForcedSteerReleaseFlag=7 生效，必要时把 kRingExitSteerError 调得更反向一点，或略降 kRingMaxDiff。\n");
    printf("  - 如果出环后一把反打出界：减小 kRingExitSteerError 的反向幅度，例如 -10 -> -6。\n\n");
}

} // namespace left_ring_exit_recommend_demo

int lq_left_ring_exit_recommend_demo(void) {
    left_ring_exit_recommend_demo::print_recommendation_card();

    if (left_ring_exit_recommend_demo::kRunHandPushFirst) {
        printf("[MODE] 先执行 T08H 手推图传模式，电机关闭。\n");
        int ret_h = lq_param_test_08h_element_push_image_debug();
        printf("[DEMO34] T08H ret=%d；若图像补线方向正确，再把 kRunHandPushFirst=false 跑自动 T08。\n", ret_h);
        return ret_h;
    }

    printf("[MODE] 执行 T08 自动低速元素调试。\n");
    int ret = lq_param_test_08_element_debug();
    printf("[DEMO34] T08 ret=%d。若仍兜圈，请发 ring=1/7/8 附近日志。\n", ret);
    return ret;
}
