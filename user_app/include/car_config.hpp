#ifndef LQ_CAR_CONFIG_HPP_
#define LQ_CAR_CONFIG_HPP_

#include "lq_camera_ex.hpp"

namespace car_config
{
// =====================================================================
// 电机硬件与基础三串级 PID 参数
// 速度单位: m/s (车轮线速度)
// 换算: m/s = 电机RPS * (30/68) * pi * 0.064 ~= RPS * 0.08870
// =====================================================================
constexpr float kEncoderRpsToMS = 0.08870f;
constexpr int kMotorPwmFrequencyHz = 8000;
// DRV8701E uses PH/EN control: PWM drives EN directly.
// ATIM mode 1 makes software duty match output high duty:
// duty 0 keeps EN low and brakes/stops; duty 10000 is full drive.
constexpr int kMotorPwmStopDuty = 0;
constexpr int kMotorPwmFullScale = 10000;
constexpr int kEncoderDiagPulsePwm = 2500;
constexpr int kEncoderDiagPulseMs = 200;
constexpr int kMotorInitialDuty = kMotorPwmStopDuty;
constexpr int kGpioProtectHeartbeatIntervalMs = 10;
constexpr int kWatchdogHeartbeatStallTimeoutMs = 100;

// [3/3] 左右轮速度 PID: 轮速目标 -> PWM (位置式)
constexpr float kSpeedPidP = 1200.0f;
constexpr float kSpeedPidI = 300.0f;
constexpr float kSpeedPidD = 30.0f;
// 地面转弯负载需要较强欠速前馈；实测连续低前馈会使外轮欠速、实际差速仅实现指令约22%。
constexpr float kSpeedPidFeedForwardPerMS = 4500.0f;
constexpr float kSpeedPidIntegralLimit = 8.0f;     // Integral state limit; output contribution is I * limit.
constexpr float kSpeedPidSlewRate = 800.0f;        // 输出变化率限制(每帧最大变化)，防止PWM突变冲击驱动板
constexpr int kMotorMaxPwm = 8000;
// DRV8701E 的 EN=0 已是低侧慢衰减制动；直道/外轮不主动反转。
// 明确弯道的内轮被车身拖行时，允许有限反拖以建立必要的左右轮差速。
constexpr float kSpeedBrakeThreshold = 0.03f;
constexpr float kSpeedBrakeMaxPwm = 0.0f;
constexpr float kSpeedTurnBrakeMaxPwm = -1200.0f;
constexpr float kBaseSpeedGoal = 0.94f;           // 激进提速：直线目标相对0.42基线提高约76%，用短时实跑逐级验证。
constexpr int kStartButtonPin = 44;                // 母板按键0：GPIO44（UART2_TX复用脚，运行时切回GPIO输入）。
constexpr int kStartButtonActiveLevel = 1;         // 原板按键电路与旧代码均为高电平按下。
constexpr int kStartButtonDebounceMs = 35;         // 释放、按下均需稳定35ms，过滤机械抖动。
constexpr int kStartButtonPollMs = 5;
constexpr int kStartupStraightDurationMs = 1000;  // car模式首个有效视觉帧后先直行1s，再交还循迹/元素控制。
constexpr float kStartupStraightSpeedGoal = kBaseSpeedGoal; // 启动直行沿用当前已验证的直道速度。
constexpr float kMaxWheelSpeedGoal = 4.0f;

// [1/3] 视觉外环 PID: 中线偏差 -> 目标角速度
// Kp×kGyroPidP必须 < kGyroPidOutputLimit/max_err，否则差速饱和
// 1.0×0.080=0.080, err=10时差速=0.80刚好不饱和
constexpr float kSteerKp = 2.55f;
constexpr float kSteerKi = 0.0f;
constexpr float kSteerKd = 0.8f;
constexpr float kSteerIntegralLimit = 80.0f;
constexpr float kSteerOutputLimitDps = 60.0f;
constexpr int kSteerDeadBand = 2;
// 远端 Det 已回中但车身仍横向偏离时，用可信近端中心补足出弯姿态回正；不参与十字/丢线帧。
constexpr float kNearCenterSteerGainDpsPerPx = 1.10f; // 115031普通弯仍有约7.9px偏差；提高可信近端位置的基础回中权重。
constexpr float kNearCenterSteerLargeErrorThresholdPx = 7.0f; // 110245弯中外切稳定在8~10px；只对已确认的大横向偏差增强回中。
constexpr float kNearCenterSteerLargeErrorGainDpsPerPx = 1.80f; // 十字前偏差会在约0.5s内快速增大，需立即补足而不能等待持续项建立。
constexpr float kNearCenterPersistentEnterErrorPx = 6.0f; // 113036稳态外切约8.6px；持续达到6px才开始确认，拒绝普通小误差。
constexpr float kNearCenterPersistentExitErrorPx = 4.0f; // 补偿建立后保留到4px，避免在6px阈值附近反复启停。
constexpr float kNearCenterPersistentHeadingMaxPx = 3.0f; // 近远中心近似平行才判为横向稳态偏置，不在入弯姿态变化期抢控制。
constexpr float kNearCenterPersistentFarErrorMaxPx = 12.0f; // 急弯/输出接近饱和时不继续积累额外角速度。
constexpr int kNearCenterPersistentSideLossMin = 12; // 仅在弯内侧远端边界持续丢失、弯向明确时建立曲率补偿。
constexpr int kNearCenterPersistentOtherLossMax = 5;
constexpr int kNearCenterPersistentConfirmMs = 250;
constexpr float kNearCenterPersistentRiseDpsPerSec = 8.0f; // 115031在6dps顶限维持约4s仍偏6.4px，允许更快建立所需曲率。
constexpr float kNearCenterPersistentDecayDpsPerSec = 32.0f; // 10dps上限仍约0.31s衰减完；方向冲突/NF则立即清零。
constexpr float kNearCenterPersistentLimitDps = 10.0f;
constexpr float kNearCenterSteerErrorLimitPx = 12.0f;
constexpr float kNearCenterSteerDeadBandPx = 2.0f;
constexpr int kNearCenterSteerMaxOffline = 20;
// 近/远中线反向且远端边界明显丢失时，说明前瞻中心可能落到横向白区或错误支路。
// 这只是几何置信度降权，不识别道路元素；近端双边仍可信时继续沿当前中线通过。
constexpr float kNearFarConflictMinFarErrorPx = 6.0f;
constexpr int kNearFarConflictSideLossMin = 12;
constexpr float kNearFarConflictFarScale = 0.0f; // 歧义期完全忽略错误远端，只沿可信近端中线继续。
constexpr int kNearFarConflictMaxHoldMs = 2300;  // 防止状态永久锁存；正常情况由连续走廊恢复提前释放。
constexpr int kNearFarConflictNearHoldMs = 100;  // 近端短暂失效时保持最后可信误差，跨过单帧/短时白区。
constexpr int kNearFarConflictNearDecayMs = 150; // 保持期后平滑衰减到直行，避免用陈旧近端持续转向。
constexpr int kNearFarConflictReleaseCenterGapPx = 5; // 前瞻/近端中心重新接续到同一纵向走廊才允许释放。
constexpr int kNearFarConflictOutletGapMaxPx = 18; // 错误方向翻转且出口走廊有效时允许较大远近差，提前跟随出十字弯。
constexpr int kNearFarConflictReleaseTowWidthMinPx = 10;
constexpr int kNearFarConflictReleaseTowWidthMaxPx = 42; // TowPoint=18期望宽约26px；拒绝横向支路的超宽假走廊。
constexpr int kNearFarConflictReleaseFrames = 3; // 连续多帧确认，不能因一帧边界计数恢复就交还远端。
constexpr float kNearFarConflictEntryHoldRatio = 0.80f; // 入口右偏10~11px时保留8~8.8px，确保十字核心回中量越过6dps差速死区。
constexpr int kNearFarConflictRecoveryBlendMs = 200; // 缩短出口交权，兼顾平滑与及时回中；实测250ms会让车身继续贴右侧行驶。
constexpr int kNearFarConflictRecoveryGuardMs = 600; // 交权后短时核验前瞻走廊；只在走廊再次失效时恢复近端保护。

// [2/3] 角度/角速度 PID: 目标角速度 -> 左右轮差速
constexpr float kAnglePidP = 11.0f;
constexpr float kAnglePidI = 0.0f;
constexpr float kAnglePidD = 0.1f;
constexpr float kAnglePidIntegralLimit = 100.0f;
constexpr float kAnglePidOutputLimit = 120.0f;
constexpr float kAngleTargetToleranceDeg = 1.2f;
constexpr float kAngleSettleGyroDps = 5.0f;

// 新双路驱动板的主动反拖更强；降低角速度反馈占比，避免实测角速度轻微过冲时差速反号。
constexpr float kGyroPidP = 0.007f;
constexpr float kGyroPidI = 0.0f;
// 实车日志里 5ms 周期下 gyro D 项会把小幅陀螺噪声放大成满幅交替差速，先关闭。
constexpr float kGyroPidD = 0.0f;
constexpr float kGyroPidIntegralLimit = 50.0f;
constexpr float kGyroPidOutputLimit = 0.80f;

// 差速输出调理参数
constexpr float kGyroFeedforwardGain = 0.0032f;  // 目标角速度前馈(m/s per dps)：以稳定同向差速为主，反馈只修正剩余误差
constexpr float kDiffSlewRatePerSec = 5.0f;       // 差速输出变化率限制(m/s²)，8ms帧约0.040m/s，抑制逐帧翻转自激
constexpr float kDiffTurnBuildSlewRatePerSec = 9.0f; // 普通弯道同方向建立转向时加快差速。
constexpr float kDiffTurnReleaseSlewRatePerSec = 12.0f; // 普通弯道出弯卸差速/反向纠偏更快，减少出弯外抛。
constexpr float kDeadbandDiffLimit = 0.05f;       // 近直线小角度目标时的差速硬上限，防止快速回中后振荡。
constexpr float kDeadbandDiffTriggerDps = 6.0f;   // 仅限制近零角速度；保留出弯 3~4 像素偏差所需的回正差速。

// 角速度环信号调理参数
constexpr float kGyroRawSpikeRejectDps = 120.0f;   // 实车目标上限60dps；超过2倍目标的读数按电机EMI尖峰拒绝。
constexpr float kGyroMeasurementLimitDps = 260.0f;  // 陀螺仪测量限幅，需高于视觉目标角速度上限，否则急弯会被内部截断成假误差。
constexpr float kGyroLpfAlpha = 0.70f;              // 陀螺仪一阶低通滤波系数；0.70 在降噪和延迟间取平衡，越大延迟越小。
constexpr float kGyroDeadBandDps = 3.0f;            // 角速度死区，|gyro_z| < 3°/s 时归零，过滤 MPU6050 静止零漂。

// 速度规划参数
constexpr float kCurveSpeedGoal = 0.50f;            // 弯道目标相对0.24基线提高约108%，是本轮平均速度提升的主要来源。
constexpr float kDegradedSpeedGoal = 0.22f;         // 十字、IMU fallback保持原安全速度，不随正常赛道提速。
constexpr float kSingleSideLossSpeedGoal = 0.30f;   // 单侧严重丢边仍保留恢复转向，同时避免长期锁死在0.22m/s。
constexpr float kDetStuckSpeedGoal = 0.26f;         // det持续偏差阶段小幅提速，但低于普通弯道目标。
constexpr float kCrossExitSpeedGoal = 0.26f;        // 十字释放后先限制车速，让出口转向角速度建立。
constexpr int kCrossExitSpeedHoldMs = 160;          // 出口短时接管窗口；只设速度上限，不削弱转向。
// 环岛元素闭环：入口/出口判据继承历史实车通过版本，帧数按当前约141~146fps换算。
constexpr int kElementRingConfirmFrames = 4;        // 连续约28ms确认入口，过滤单帧跳边。
constexpr int kElementCooldownFrames = 180;         // 调试帧率下约3s不重复识别同一环岛；正式帧率下约1.2s。
constexpr int kRingRedetectLockFrames = 600;         // 出环后约4s禁止再次识别环岛，确保车辆完全离开同一圆环。
constexpr int kRingAbortRedetectLockFrames = 180;    // 误判撤销只短锁约1.2s；避免跳过紧接U弯后的真实环岛。
constexpr int kRingCrossExitLockFrames = 520;        // 十字核心后约3.5s禁止仅凭单侧丢边触发环岛，覆盖十字出口支路。
constexpr int kRingCrossExitClearFrames = 90;        // 最短锁定结束后还需约0.6s稳定双边车道，避免在出口弯中途解锁。
constexpr int kElementRingExitStraightHoldFrames = 30; // 出环后继续锁直约0.2~0.5s，避免尚未摆正便重新加速。
constexpr int kRingApproachCenterBias = 0;          // 经过第一个开口前沿可信外侧边界走本车道，不向出口预偏。
constexpr int kRingEntryCenterBias = 7;             // 切入阶段中线明确指向环心；远小于旧版30px硬拉偏置。
constexpr int kRingInnerOuterBias = 0;              // 环内严格沿可信外侧边界减半幅，禁止累计向环外漂移。
constexpr int kRingInnerCenterBias = kRingInnerOuterBias; // 参数显示兼容名；实际方向由环内外侧补线函数决定。
constexpr int kRingExitCenterBias = 0;              // 出环仍保持真实车道中线；旧8px外偏会直接产生反向转向。
constexpr int kLeftRingRightLineBlockMax = 12;
constexpr int kLeftRingOfflineBlockMax = 18;
constexpr int kLeftRingMinLeftLost = 8;
constexpr int kLeftRingMaxPoint2Y = 42;
constexpr int kRightRingLeftLineBlockMax = 18;
constexpr int kRightRingOfflineBlockMax = 25;
constexpr int kRightRingMinRightLost = 6;
constexpr int kRightRingMinPoint2Y = 15;
constexpr int kRingLossOnlyMinLost = 28;
constexpr int kRingLossOnlyMaxEntryError = 10;
constexpr int kLeftRingLossOnlyMaxSignedEntryError = 2; // 仅丢左边兜底不接受明确左弯；大弯实测+5，真环几何触发约-2。
constexpr int kRightRingLossOnlyMinSignedEntryError = -2; // 仅丢右边兜底不接受急右弯；误判实测-7，真小右环触发为0。
constexpr float kRingSpeedGoal = 0.36f;             // 首轮保守环岛速度，低于普通弯道0.50m/s。
constexpr float kRingExitSpeedGoal = 0.32f;         // 出环冷却期禁止立刻跳回0.68m/s直线速度。
constexpr float kRingEntryGyroLimitDps = 38.0f;
constexpr float kRingSceneGyroLimitDps = 52.0f;
constexpr float kRingExitGyroLimitDps = 26.0f;       // 小左环出口目标已落在车头左侧时保留额外同向曲率，减少出口外切。
constexpr float kRingExitAlignGyroLimitDps = 26.0f;  // 阶段9误差长期顶住24dps且实车轻微出界；提高2dps加快回到出口中线。
constexpr float kRingExitOppositeGyroLimitDps = 6.0f; // 出口补线若瞬时给出反向，只允许低幅纠偏，禁止再次反打。
constexpr float kRingExitAlignOppositeGyroLimitDps = 12.0f; // 163832出口中线已右转但raw=-39被压成-4，实车近似直行；12dps保留受限出环转向。
constexpr float kRingApproachMinGyroDps = 0.0f;      // 第一个开口是出口；经过它之前禁止强制向环心转向。
constexpr float kRingTurnInMinGyroDps = 36.0f;       // 165502入口误差仅-1px仍固定38、实测43dps并轻微内切；小降2dps保留可靠切入。
constexpr float kRingInnerMinGyroDps = 36.0f;        // 30dps时环内长期外切；取与旧42dps贴内线版本的中间值。
constexpr float kRingInnerAdaptiveEarlyMinGyroDps = 24.0f; // 前半圈保留曲率下限，避免刚入大环时进一步外切。
constexpr float kRingInnerAdaptiveLateMinGyroDps = 12.0f;  // 后半圈允许视觉把持续内切平滑拉回中线。
constexpr float kRingInnerAdaptiveMaxGyroDps = 40.0f;      // 小环外切时多保留1dps曲率；回到内侧后仍由自适应下限自动释放。
constexpr float kRingInnerOutwardGainDpsPerPx = 2.5f;      // 目标位于环外时更快释放曲率，但不越过阶段安全下限。
constexpr float kRingInnerInwardGainDpsPerPx = 0.5f;       // 目标位于环内时渐进增加曲率，消除36到39的阈值跳变。
constexpr float kRingInnerLateReliefYawDeg = 210.0f; // 仅后半圈启用更强释放，避免加重大环入口外切。
constexpr int kRingInnerLateReliefMinMs = 4200;     // IMU短暂停时转角会少计；阶段时长提供不依赖陀螺仪的后半圈门控。
constexpr int kRingPhaseConfirmFrames = 4;           // 所有阶段转换统一连续确认，禁止单帧跳态。
constexpr int kRingApproachLowerLostMin = 6;         // 第一个开口进入近端视野，转入“经过开口”阶段。
constexpr int kRingApproachMinMs = 80;               // 过滤刚锁定环岛时的单帧边界跳变。
constexpr int kRingApproachForcePassMs = 400;        // 漏掉开口前沿时也进入经过开口阶段，不直接强转。
constexpr int kRingPassOpeningPeakMissingMin = 6;    // 145322实测近端峰值仅6；达到后再等待恢复到4，锁定正确入口时机。
constexpr int kRingPassOpeningRecoveredMax = 4;      // 近端内侧边界恢复，说明第一个开口已经从车侧后移。
constexpr int kRingPassOpeningMinMs = 300;           // 至少保持本车道300ms，禁止再次从出口切入。
constexpr int kRingPassOpeningForceSeekMs = 1100;    // 第一开口恢复特征漏检时只转入寻找入口，绝不直接强转。
constexpr int kRingSeekEntranceMinMs = 220;          // 第一开口闭合后继续补线，过滤同一边缘在前瞻行附近的抖动。
constexpr int kRingEntranceLowerLostMin = 4;         // 第二开口必须从远端前瞻行扩展到近端，禁止tow=0/lower=0时提前切入。
constexpr int kRingEntranceLineLostMin = 24;         // 154436入口在丢24行时已稳定展开；无需再直行到近端丢4行。
constexpr int kRingSeekEntranceAbortMs = 1300;       // 第二开口始终无近端证据时撤销误判；禁止U弯超时后反向强转。
constexpr int kRingEnterOuterLostHigh = 10;          // 切入时对侧边界消失，说明车头已穿过入口切线。
constexpr int kRingEnterOuterLostLow = 10;           // 沿用实车成功条件<10，并以连续4帧确认对侧边界恢复。
constexpr int kRingEnterLossWaitMaxMs = 1500;        // 边界特征漏检时也不能永久卡在进环前半段。
constexpr int kRingEnterRecoverMaxMs = 1200;         // 对侧恢复漏检的保守超时，仅用于防状态锁死。
constexpr int kRingInnerMinStageMs = 900;            // 阶段7至少维持0.9s环内转向，再允许搜索结果切到出环。
constexpr int kRingOrbitExitGeometryMinMs = 6000;    // 165055无IMU异常仍在5.0s/yaw235度过早释放；延后约1s补足至约272度再接出口。
constexpr float kRingOrbitExitMinYawDeg = 330.0f;    // 154159在292度按角点退出后仍处环弯并丢线；角点也必须完成约330度转角。
constexpr int kRingOrbitExitInnerLostMin = 12;       // 真出口需保持外侧车道可跟踪且内侧开口展开；挡住大环off=20/inner=0假出口。
constexpr float kRingOrbitForceExitYawDeg = 380.0f;  // 几何证据始终漏检时才晚兜底；不再于330度无条件离开大半径弧线。
constexpr int kRingExitSearchMinMs = 220;            // 阶段8至少跟随出口补线220ms，禁止看见一帧直线就释放。
constexpr int kRingExitSearchMaxMs = 1200;           // 出口边界漏检时转入低幅对正，禁止阶段8无限持续。
constexpr int kRingExitAlignMinMs = 320;             // 阶段9至少完成320ms出口对正。
constexpr int kRingExitAlignMaxMs = 900;             // 对正条件持续缺失时转入锁直，避免无限反打。
constexpr int kRingExitCenterErrorMax = 6;           // 出口前瞻回到中线附近才允许正常完成。
constexpr int kRingAlignInnerLossMax = 10;            // 对正阶段内侧边界恢复后立即改用真实双边界中线。
constexpr float kRingEntryMaxDiff = 0.24f;          // 入口至少保留约0.12m/s内轮目标。
constexpr float kRingMaxDiff = 0.30f;               // 环内允许更大曲率但避免内轮近停。
constexpr float kSpeedAccelLimit = 4.0f;            // 缩短出弯再加速时间，避免长时间停留在弯道速度。
constexpr float kSpeedDecelLimit = 4.0f;            // 目标速度下降限幅 m/s^2，0.45→0.20 仅需 62ms，入弯减速足够快。
constexpr float kSpeedPlanSlowStart = 3.0f;         // 小偏差保持直线速度，减少轻微摆动导致的无效刹车。
constexpr float kSpeedPlanFullCurve = 14.0f;        // 只有明确弯道才降至完整弯道速度。
constexpr float kSpeedPlanDerivWeight = 0.15f;      // 降低瞬时偏差变化率对刹车的放大，缩短弯前低速区。
constexpr float kVisionLossSlowStart = 14.0f;       // 轻微边界缺失不再过早降速。
constexpr float kVisionLossFullCurve = 34.0f;       // 严重视觉退化时才降到完整弯道速度。

// 元素总开关保持开启：当前竞赛版需要环岛、十字等元素处理。
// 该兼容开关只控制元素处理，不再承担日志/图传开关职责。
constexpr bool kNoElementCompetitionMode = false;
// 纯竞赛静默模式：关闭调试日志、8080 遥测、8081 JPEG 图传和本地竞赛帧采集，
// 仅保留启动信息以及相机、IMU、编码器、看门狗等故障安全提示。
constexpr bool kCompetitionQuietMode = true;
constexpr bool kSteerDebugPrintEnable = !kCompetitionQuietMode;
constexpr bool kElementDebugLogEnable = !kCompetitionQuietMode;
constexpr bool kRaceFrameCaptureEnable = !kCompetitionQuietMode;
constexpr int kSteerDebugPrintDivider = 10;

// MPU6050 量程换算与零偏标定参数
constexpr float kMpu6050AccelScale = 16384.0f;   // 加速度量程换算，当前对应正负 2g；如果驱动量程改了必须同步改，否则加速度数值会错。
constexpr float kMpu6050GyroScale = 16.4f;       // 陀螺仪量程换算，当前对应正负 2000dps；如果驱动配置不是该量程，角速度闭环会整体失真。
constexpr int kMpu6050GyroBiasSamples = 200;     // 陀螺仪零偏采样次数，调大零偏更稳但启动更慢；零偏波动大可加到 300-500。
constexpr int kMpu6050BiasSampleDelayUs = 10000; // 零偏采样间隔，10000us 即 10ms；采样太快噪声相关性高，建议保持 5-10ms。

// =====================================================================
// 摄像头参数：高速赛道模式采用160x120原生帧；当前NCNN观测关闭。
// =====================================================================
constexpr int kCameraHiresWidth = 160;           // 180fps档位使用160x120；循迹原生即为该分辨率。
constexpr int kCameraHiresHeight = 120;          // 320x240会被驱动协商回120fps，因此使用传感器高速档。
constexpr int kCameraWidth = 160;                // 循迹下采样宽度，保持与既有算法兼容；修改此值需同步调整 ROI 和采样步长。
constexpr int kCameraHeight = 120;               // 循迹下采样高度。
constexpr int kCameraFps = 180;                  // 请求180fps；新 DHZJ 摄像头在 0CPU 模式下实际协商为 YUYV 156fps。
constexpr lq_camera_format_t kCameraFormat = LQ_CAMERA_0CPU_MJPG; // 赛道测试配置：低 CPU 枚举；实测约141~146fps，请勿按180fps估算处理周期。
constexpr const char *kCameraDevicePath = LQ_CAMERA_PATH; // 摄像头设备节点，默认 /dev/video0；换摄像头编号时改这里。
constexpr int kCameraManualExposure = 55;       // T03白占比0.563偏高，先降到历史安全甜点55，优先压过曝/反光导致的动态丢边。

// =====================================================================
// 红色标记检测参数（12x5cm 红色标记锚点，用于定位上方 12x12cm 目标区域）
// =====================================================================
constexpr int kRedMarkerHsvLowH1 = 0;           // 红色 HSV 主区间 H 下限。
constexpr int kRedMarkerHsvLowS1 = 120;         // 红色 HSV 主区间 S 下限。
constexpr int kRedMarkerHsvLowV1 = 80;          // 红色 HSV 主区间 V 下限。
constexpr int kRedMarkerHsvHighH1 = 8;          // 红色 HSV 主区间 H 上限。
constexpr int kRedMarkerHsvHighS1 = 255;        // 红色 HSV 主区间 S 上限。
constexpr int kRedMarkerHsvHighV1 = 255;        // 红色 HSV 主区间 V 上限。
constexpr int kRedMarkerHsvLowH2 = 172;         // 红色 HSV 副区间 H 下限（红色在 HSV 空间跨 0 度边界）。
constexpr int kRedMarkerHsvLowS2 = 120;         // 红色 HSV 副区间 S 下限。
constexpr int kRedMarkerHsvLowV2 = 80;          // 红色 HSV 副区间 V 下限。
constexpr int kRedMarkerHsvHighH2 = 180;        // 红色 HSV 副区间 H 上限。
constexpr int kRedMarkerHsvHighS2 = 255;        // 红色 HSV 副区间 S 上限。
constexpr int kRedMarkerHsvHighV2 = 255;        // 红色 HSV 副区间 V 上限。
// 底部红色横条回退检测（放宽阈值）
constexpr int kRedBandHsvLowH1 = 0;             // 红色横条回退 H 下限。
constexpr int kRedBandHsvLowS1 = 60;            // 红色横条回退 S 下限（放宽）。
constexpr int kRedBandHsvLowV1 = 40;            // 红色横条回退 V 下限（放宽）。
constexpr int kRedBandHsvHighH1 = 20;           // 红色横条回退 H 上限（放宽）。
constexpr int kRedBandHsvHighS1 = 255;          // 红色横条回退 S 上限。
constexpr int kRedBandHsvHighV1 = 255;          // 红色横条回退 V 上限。
constexpr int kRedBandHsvLowH2 = 165;           // 红色横条回退副区间 H 下限。
constexpr int kRedBandHsvLowS2 = 60;            // 红色横条回退副区间 S 下限。
constexpr int kRedBandHsvLowV2 = 40;            // 红色横条回退副区间 V 下限。
constexpr int kRedBandHsvHighH2 = 180;          // 红色横条回退副区间 H 上限。
constexpr int kRedBandHsvHighS2 = 255;          // 红色横条回退副区间 S 上限。
constexpr int kRedBandHsvHighV2 = 255;          // 红色横条回退副区间 V 上限。
constexpr float kRedMinArea = 20.0f;            // 红色标记最小像素面积。
constexpr float kRedAspectLow = 1.8f;           // 红色标记宽高比下限。
constexpr float kRedAspectHigh = 4.5f;          // 红色标记宽高比上限。
constexpr float kRedAreaRatioLow = 0.001f;      // 红色面积占全图比例下限。
constexpr float kRedAreaRatioHigh = 0.15f;      // 红色面积占全图比例上限。
constexpr float kRedFillRatioMin = 0.55f;       // 红色轮廓填充率下限。
constexpr float kRedCenterYMinRatio = 0.30f;    // 红色标记中心 Y 占比下限（忽略顶部红色物体）。
constexpr int kRedMorphCloseIter = 2;           // 红色掩膜形态学闭合迭代次数。

// =====================================================================
// NCNN 推理参数（走马观碑组：识别 12x12cm 目标区域内的文字/符号）
// 模型: TinyClassifier (width_mult=0.6), 48×48, 三分类
// 训练归一化: mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225] (ImageNet)
// 对应 NCNN 参数: mean_vals = mean×255, norm_vals = 1/(std×255)
// =====================================================================
constexpr int kNcnnCropSide = 48;               // 从红色标记上方裁剪的正方形边长（像素）。
constexpr int kNcnnInputWidth = 48;             // NCNN 模型输入宽度（与训练时 --img-width 一致）。
constexpr int kNcnnInputHeight = 48;            // NCNN 模型输入高度（与训练时 --img-height 一致）。
constexpr int kNcnnInferInterval = 10;          // 每 N 帧主循环执行一次 NCNN 推理（降低 CPU 负载）。
constexpr float kNcnnConfidenceThreshold = 0.90f; // NCNN 推理置信度阈值，低于此值的结果视为无效。
constexpr const char *kNcnnModelParam = "tiny_classifier_fp32.ncnn.param";  // run_all.ps1 产物。
constexpr const char *kNcnnModelBin = "tiny_classifier_fp32.ncnn.bin";      // run_all.ps1 产物。
constexpr int kNcnnObjectBottomGap = 0;         // NCNN观测T: 下移ROI验证物资图案是否因gap=5过低而被误判。
// 标签列表（与 ImageFolder 子目录名一致，按字母排序）
constexpr const char *kNcnnLabel0 = "supplies";
constexpr const char *kNcnnLabel1 = "vehicles";
constexpr const char *kNcnnLabel2 = "weapons";
constexpr bool kNcnnObserveEnable = false;      // 速度调试阶段暂时关闭 NCNN 观测与推理。
constexpr int kNcnnObserveUdpEveryFrames = 10;  // 每 N 帧提交一次异步观测，避免推理拖慢图像主循环。
constexpr int kNcnnObserveJpegQuality = 55;     // UDP 图传 JPEG 质量。
constexpr const char *kNcnnObserveUdpHost = "192.168.145.22"; // 上位机 UDP 图传地址。
constexpr int kNcnnObserveUdpPort = 8081;       // 上位机 UDP 图传端口。
constexpr const char *kRaceTelemetryUdpHost = "192.168.145.22"; // 上位机转向遥测地址。
constexpr int kRaceTelemetryUdpPort = 8080;     // 上位机转向遥测端口。
constexpr int kCameraRoiX = 0;                   // ROI 左上角 X，通常保持 0；摄像头画面横向偏移明显时先机械校正，再微调。
constexpr int kCameraRoiY = 58;                  // T06入弯后OFF/单侧丢边高，ROI略前移2行多看远端，避免动态闭环看太近。
constexpr int kCameraRoiWidth = 160;             // ROI 宽度，应与采集宽度和压缩逻辑匹配；非必要不要改。
constexpr int kCameraRoiHeight = 60;             // ROI 高度，越大信息更多但处理慢；若远端元素判断不足可适当增大。
constexpr int kCameraColumnSampleStep = 2;       // 横向采样步长，2 表示 160 压缩到 80 列；调大更快但精度低，调小要同步修改图像数组宽度。
constexpr int kBlueHueLower = 90;                // 蓝色背景 H 下限，背景被漏检时降低，黑线被误当蓝色时提高。
constexpr int kBlueSaturationLower = 80;          // T03/T06：当前实际走HSV链路，S=100边界/单侧丢失偏多，先放宽到历史img_bad较低的80。
constexpr int kBlueValueLower = 60;              // 蓝色背景 V 下限，暗场漏检时降低，阴影噪点误检时提高。
constexpr int kBlueHueUpper = 130;               // 蓝色背景 H 上限，背景偏紫/偏青时调整；上限过高可能吞掉非赛道区域。
constexpr int kBlueSaturationUpper = 255;        // 蓝色背景 S 上限，通常保持 255；只有特殊反光场景才收窄。
constexpr int kBlueValueUpper = 255;             // 蓝色背景 V 上限，通常保持 255；强反光误检严重时可略降。

// 图像二值化算法选择
constexpr int kImageAlgorithm = 1;               // T06弯道OTSU丢边严重：OTSU在弯道暗区比例变化时阈值跳变，切换到静态阈值(130)更稳定。
constexpr int kImageOtsuBlurKernel = 3;          // OTSU前灰度滤波核大小，只支持奇数；3可抑制噪点且不明显糊边。
constexpr int kImageMorphKernel = 1;             // T06动态弯道单侧丢边明显；先关闭3x3形态学，避免开运算吞细白线/边界。

// 图像循迹与元素判断阈值
constexpr int kImageMiddleLine = 38;             // 图像压缩后中心列；T13手推直道minL=13比minR=15少2，向右修正1列平衡左右margin。
constexpr int kImageTowPoint = 18;               // 实车右直角会看到邻近赛道，前瞻拉回近场，优先追本车道中线。
constexpr int kImageThresholdStatic = 130;       // 灰度二值化阈值；OTSU模式下不参与主链路，保留备用。
constexpr int kImageThresholdDetach = 180;       // 阳光/亮斑分离阈值备用参数，强反光误判时提高，暗场丢线时降低。
constexpr int kImageScanInterval = 4;            // 动态弯道边界断续时放宽普通扫边1格，优先提高找回边界能力。
constexpr int kImageScanIntervalMin = 3;         // 直道/低误差时收窄扫边窗口，减少误扫到对侧边界。
constexpr int kImageScanIntervalMax = 7;         // 丢线/大偏差时放宽扫边窗口，提升急弯找回边界能力。
constexpr int kImageScanDynamicLossStart = 6;    // 单侧丢线累计达到该值后逐步放宽扫线窗口。
constexpr int kImageScanIntervalCross = 4;       // 十字/复杂元素扫边范围，调大更容易找回边但更易误补线；本次收紧，减少十字路口连转误判。
constexpr int kImageVariance = 90;               // 直道判断方差阈值，越大越容易判直道；直道不加速可增大，弯道误判直道则降低。
constexpr int kImageVarianceAcc = 45;            // 直线加速检测阈值备用参数，越大越容易进入加速；确认直线判断可靠后再放宽。
constexpr int kBadVisionCenterHoldOfflineThreshold = 55; // 十字/近乎全丢线时不沿用旧偏差，先按中线低速通过。
constexpr int kBadVisionDetHoldOfflineThreshold = 35; // 十字入口中线可能先跳到支路；补线未成功时提前保持上一有效方向。

// 十字路口角点补线参数
constexpr int kCrossMinWhiteLine = 8;             // 触发十字角点检测的最小双丢边行数；过小会误触发普通弯道。
constexpr int kCrossCornerScanStartRow = 10;      // 角点搜索起始行（从近端开始）。
constexpr int kCrossCornerScanEndRow = 50;        // 角点搜索终止行（远端截止）。
constexpr int kCrossCornerMinBorderLen = 5;       // 角点检测需要的连续有边行数最小值；过小会把噪点当角点。
constexpr int kCrossRepairSlopeLimit = 2.0f;      // 补线斜率绝对值上限，防止补线角度过大飞掉。
constexpr int kCrossMinCornerYGap = 5;            // 上下角点之间最小行距，过滤太近的伪角点。
constexpr int kCrossRepairOfflineExtend = 3;      // 补线后OFFLine向上扩展行数，确保GetDet能看到补好的线。
constexpr int kCrossHoldEnterSideLossMin = 12;    // 十字入口双侧弱观测阈值；与弯道中心线参数解耦，避免调弯道时暗改锁存入口。
constexpr int kCrossHoldCoreWhiteLineMin = 16;    // 白线宽度达到该值后确认已经进入十字核心；实测入口预兆5~9、核心峰值34~41。
constexpr int kCrossHoldReleaseOfflineMax = 20;   // 十字入口锁向后，OFFLine恢复到该值以下才允许开始释放。
constexpr int kCrossHoldReleaseWhiteLineMax = 3;  // 双丢边行基本消失后才认为已经越过十字核心区。
constexpr int kCrossHoldReleaseSideLossMax = 10;  // 双边恢复或越过核心后任一侧恢复到该值以下，允许重新接管出口弯。
constexpr int kCrossHoldReleaseFrames = 3;         // 实测约117fps时约26ms，减少十字出口欠转，同时过滤一帧假恢复。
constexpr float kCrossSteerErrorLimit = 4.0f;      // 十字锁向只保留小姿态修正，防止侧支路把车持续拉向岔口。
constexpr float kCrossSteerKd = 0.8f;              // 十字保持已验证的原始D响应；普通弯道继续使用快速回中参数。
constexpr int kCrossEntryHeadingNearRow = 55;      // 用近端与前瞻中心差估计出弯后的车头姿态残差。
constexpr int kCrossEntryHeadingMaxOffline = 20;  // 只记忆边界仍可信的弯道末端姿态，避免十字坏帧污染参考。
constexpr float kCrossEntryHeadingLimitPx = 10.0f; // 限制入口姿态残差，防止单边补线异常造成过度反打。
constexpr int kCrossEntryHeadingMemoryMs = 160;    // 十字触发前短时保留最后可信姿态，覆盖入口瞬间的丢边帧。
constexpr int kCrossEntryAlignDurationMs = 220;    // 首次进入十字后的短时对正窗口，不贯穿整个交叉区。
constexpr float kCrossEntryHeadingGainDps = 1.6f; // 每像素近远中心差对应的反向对正角速度。
constexpr float kCrossEntryHeadingLimitDps = 16.0f;// 对正角速度上限；足以消除残余偏航，又低于正常急弯目标。
constexpr int kCrossEntryRearmNormalFrames = 18;   // 连续正常约150ms后才允许下一次入口触发，过滤Road_type抖动。
constexpr int kStraightVarianceAccThreshold = 12; // 静态直道 var≈9.96 已贴近阈值，略放宽到 12，避免直道在边界处反复 Normol/Straight。
constexpr int kStraightOfflineMax = 8;           // 直道允许的顶边丢线数，调大更容易判直道但可能误判弯道；丢线多时先查二值化再改。
constexpr int kStraightLeftLineMax = 3;          // 直道允许的左丢边数，调大更宽松；左边界不稳定时先调曝光和 HSV，再微调。
constexpr int kStraightRightLineMax = 3;         // 直道允许的右丢边数，调大更宽松；右边界不稳定时同样先处理图像质量。

// 稳定中心线重构：按边界可信度做补线，避免单侧丢线/宽度异常时中线乱飘。
constexpr bool kRaceUdpDebugEnable = !kCompetitionQuietMode; // 纯竞赛版不初始化异常帧 JPEG UDP 图传。
constexpr int kRaceUdpDebugMaxFrames = 1200;      // 环岛异常期会逐帧发送；覆盖A>823的完整出环与离场过程。
constexpr int kRaceUdpDebugIntervalFrames = 24;   // 120fps下约5fps，兼顾时序分辨率与JPEG编码/UDP负载。
constexpr bool kBasicLineBaselineMode = false;    // 纯中线循迹也启用单侧边界中线重构，避免弯道丢一侧后中线贴边。
constexpr bool kBasicCrossRepairEnable = false;   // 纯中线循迹不识别十字；十字与普通弯道统一依靠连续边界补线。
constexpr bool kCurveCenterCompEnable = true;     // 开启轻量级可信度补线；基础基线模式会临时跳过。
constexpr int kCurveCenterLossStart = 4;          // 单侧丢线达到该值后提高单边重构权重；过小会误伤普通小弯，过大则大弯介入偏晚。
constexpr int kCurveCenterOppositeLossMax = 12;   // 对侧也严重丢线时少信单侧边界，改走本帧行间预测，避免双边弱观测乱补。
constexpr int kCurveCenterWidthTolerance = 16;    // 宽度偏离期望赛道宽度超过该值时，即使标记为T也按不可靠边界处理。
constexpr int kCurveCenterSingleSideBias = 0;     // 保持几何中线无偏；物理外切由可信近端位置外环纠正，避免恢复历史上的提前内切。
constexpr int kCurveCenterMaxRowStep = 5;         // TFT显示/控制中线相邻行最大跳变，抑制大弯远端中线突然冲出边界。
constexpr int kCurveCenterFrameMaxStep = 4;       // 同一行跨帧最大接纳步长，吸收抖动但允许真实弯道连续移动。
constexpr int kCurveCenterBadFrameMaxStep = 1;    // 双边都弱或宽度明显异常时跨帧更保守，避免坏图像把控制目标带飞。
constexpr int kCurveCenterMinValidWidth = 10;     // 小于该宽度视为不可用，防止边界夹到一起后仍参与平均。
constexpr int kCurveCenterMaxValidWidth = 72;     // 大于该宽度视为不可用，防止扫到画面边缘造成中线跳边。
constexpr uint32_t kEncoderStalePeriodThreshold = 50000000U; // 编码器周期超时阈值（160MHz下约0.31s，即停止）
}

#endif
