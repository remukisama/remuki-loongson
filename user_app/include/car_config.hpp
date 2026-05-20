#ifndef LQ_CAR_CONFIG_HPP_
#define LQ_CAR_CONFIG_HPP_

#include "lq_camera_ex.hpp"

namespace car_config
{
// 电机硬件与最内层速度环参数
constexpr int kMotorPwmFrequencyHz = 25000;      // 电机 PWM 频率，频率过低电机会啸叫，过高会降低有效分辨率；通常保持 20k-25kHz，除非驱动器发热或响应异常。
constexpr int kMotorInitialDuty = 0;             // 电机上电初始占空比，建议保持 0；调车时不要改大，避免程序启动瞬间冲车。
constexpr float kSpeedPidP = 57.0f;              // 速度环比例项，恢复原值。
constexpr float kSpeedPidI = 26.5f;              // 速度环积分项，恢复原值。
constexpr float kSpeedPidD = 0.0f;               // 速度环微分项，恢复原值。
constexpr int kMotorMaxPwm = 8000;               // PWM 输出限幅，调大动力更强但更容易打滑和烧驱动；先确保满电、空载和上赛道都不发热再提高。
constexpr int kCurveSpeedGoal = 11;              // 弯道基础轮速目标；低速闭环已通过，先小幅提速观察 img_bad/GErr/cap 是否仍健康。
constexpr int kStraightSpeedGoal = 14;           // 直道基础轮速目标；从低速可用档进入下一轮提速验证，弯道和极限弯仍保持保守。
constexpr int kMaxWheelSpeedGoal = 100;          // 单轮目标速度上限，防止差速分配过大；若调参时目标速度经常被截断，先检查差速参数而不是直接放大。

// 视觉外环与差速分配参数
constexpr float kSteerKp = 4.5f;                  // 视觉偏差到目标角速度的 P 项；低速测试先降低转向增益，减少左右摆幅放大。
constexpr float kSteerKd = 2.0f;                  // 视觉偏差变化率 D 项；配合降低KP，减少误差快速变化时的角速度目标尖峰。
constexpr float kMaxDiffRatio = 1.55f;           // 最大差速占基础速度的比例，恢复原值。
constexpr int kSteerDeadBandDefault = 2;          // 默认视觉偏差死区，从 1 提高到 2 减少直道细碎修正；配合更低 KP 保持入弯灵敏度。
constexpr int kSteerDeadBandStraight = 2;        // 直道死区，调大可减少直道细碎修正；偏小可让入弯更早降速和打方向。
constexpr int kSteerDeadBandCurve = 1;           // 弯道死区，调小弯道更灵敏但噪声更多；本次降到 1，减小小弯迟钝。
constexpr int kMinWheelSpeedGoal = 2;            // 单轮最低目标速度，避免急弯内轮长时间停住造成拖拽/甩尾；力度不足时略放开内轮余量。
constexpr float kOuterLoopMinPeriodSec = 0.02f;  // 视觉/IMU 外环最小刷新周期，0.02s 约 50Hz；CPU 忙或 I2C 卡顿可增大到 0.03s，响应慢则减小。
constexpr float kOuterLoopFallbackPeriodSec = 0.02f; // dt 异常时使用的兜底周期，应与外环周期一致；改外环频率时同步修改。
constexpr float kSteerFilterAlpha = 0.4f;        // 视觉偏差一阶低通系数，从0.5提到0.6加快滤波跟踪，S弯换向更果断。
constexpr float kSteerRawJumpThreshold = 14.0f;  // Raw单帧突变判定阈值，兼容当前S弯跳边保护逻辑。
constexpr float kSteerRawMaxStepPerFrame = 7.0f; // Raw突变时每帧最大接纳步长，避免识别跳边直接反打。
constexpr float kSteerFilterMaxStepPerFrame = 5.2f; // Filt最终单帧最大变化量，限制滤波输出突跳。
constexpr float kSteerFilterSignFlipMaxStepPerFrame = 3.2f; // Filt跨符号换向时更严格，避免一帧大幅反打。
constexpr float kSteerFastFlipAlpha = 0.78f;     // S 弯大幅换向时的快速滤波系数，让 Filt 尽快跟上 Raw，避免继续朝旧方向打。
constexpr float kSteerFastFlipRawThreshold = 8.0f;  // 触发快速换向的 Raw 最小幅度，从 10 降到 8 让 S 弯更早触发快速跟踪。
constexpr float kSteerFastFlipGap = 15.0f;        // Raw 与 Filt 差值换向判定，从 18 降到 15 让中等换向也能触发。
constexpr int kSCurveSpeedGoal = 8;              // S弯换向速度；低速闭环阶段保守一点，避免S弯/摆动时图像贴边。
constexpr float kSCurveMinDiff = 15.0f;          // S 弯保护时保留的最小差速余量，避免入弯前段目标角速度还没拉起时差速不够。
constexpr int kSCurveHoldFrames = 16;            // S弯触发后保持保护的外环帧数，从16降到12约240ms，减少S弯保护时间。
constexpr int kExtremeTurnSpeedGoal = 6;         // 极限急弯速度；低速闭环阶段先降低贴边风险。
constexpr float kExtremeTurnMinDiff = 19.0f;     // 极限弯降速后保留的最小差速余量，贴边段给外轮更多速度，减少冲出赛道。
constexpr float kExtremeTurnExitRawThreshold = 3.0f; // 极限弯出弯判定 Raw 阈值，Raw 回到中心附近时加快滤波和目标角速度回收。
constexpr float kExtremeTurnExitFiltThreshold = 8.0f; // 极限弯出弯判定 Filt 阈值，只有滤波仍明显滞后时才触发快收。
constexpr float kExtremeTurnExitFilterAlpha = 0.82f;  // 出极限弯时临时加大的视觉滤波系数，减少 Filt 拖着旧方向继续打。
constexpr float kSCurveRawMinError = 4.0f;       // S 弯预判最小 Raw 偏差，小于该值多半是中心附近噪声；急弯出界时提前触发保护。
constexpr float kSCurveEarlyRawMinError = 1.0f;  // Raw 刚越过中心但幅度还小时，也允许配合 Filt 反向和大 dE 提前预警。
constexpr float kSCurveEarlyFiltMinError = 1.0f; // Filt 仍停在旧方向的最小幅度，过滤中心附近抖动。
constexpr float kSCurveEarlyDerivativeThreshold = 2.2f; // 小 Raw 换向预警的 dE 阈值，专门抓 S 弯初始过零帧。
constexpr float kSCurveRawFiltGap = 8.0f;        // Raw 与 Filt 差值较大说明图像已经换向但车身/滤波还在旧方向，提前降速。
constexpr float kSCurveDerivativeThreshold = 1.8f; // S 弯/急变的单帧滤波偏差变化阈值，超过后给目标角速度更快跟随。
constexpr float kSCurveSharpRawThreshold = 12.0f;  // Raw 已经较大时即使 Filt 还没跟上，也持续按 S 弯/急弯保护速度处理。
constexpr float kExtremeTurnRawThreshold = 20.0f;  // 极限贴边急弯 Raw 阈值，接近图像边缘时进一步降速；出界时提前触发。
constexpr float kExtremeTurnFiltThreshold = 18.0f; // 极限贴边急弯 Filt 阈值，防止 Raw 单帧噪声误触发；出界时提前触发。
constexpr float kSharpTurnSteerError = 10.0f;    // 大偏差判定阈值，超过后强制用弯道速度；大弯出界时提前进入降速/强转状态。
constexpr float kVisualGyroLimitDps = 200.0f;     // 视觉外环输出目标角速度限幅，从200降到180避免过大目标角速度导致内环跟踪不上。
constexpr float kTargetGyroMaxDeltaDps = 28.0f;  // 目标角速度变化率限制，从28放宽到35让目标更快建立。
constexpr float kTargetGyroSCurveMaxDeltaDps = 44.0f; // S 弯换向/Raw-Filt 大分离时放宽目标变化率，避免车身先出界再救回；过大容易连续反冲。
constexpr float kTargetGyroSignFlipMaxDeltaDps = 210.0f; // 目标角速度换向时的变化率限制，只在正负号切换时放宽；确保大幅换向先跨过零，不继续朝旧方向打。
constexpr float kTargetGyroDeadbandReturnDeltaDps = 72.0f; // 视觉进入死区后目标角速度回零速度，调大出弯姿态收得更快，过大则直道可能有回正抽动。
constexpr float kExtremeTurnExitTargetDeltaDps = 140.0f; // 极限弯出弯时目标角速度允许更快回收，防止 Raw 已回中但 Tgt 仍长时间贴顶。
constexpr float kOpposingGyroHoldDps = 24.0f;   // S弯反打时，如果车身仍在旧方向旋转，先限制目标角速度，从24降到18减少误触发。
constexpr float kOpposingGyroTargetLimitDps = 95.0f; // 反向旋转未收住时的临时目标角速度上限，从95提到130避免S弯换向时目标被过度限制。
constexpr int kOpposingGyroRecoveryFrames = 18; // 反向保护触发后继续保持几帧较低目标，从18降到10加快恢复。
constexpr float kOpposingGyroRecoveryLimitDps = 115.0f; // 反向保护解除后的短暂恢复目标上限，从115提到145加快S弯换向恢复。
constexpr bool kSteerDebugPrintEnable = true;    // 置 false 可关闭位置环/视觉调试串口打印。
constexpr int kSteerDebugPrintDivider = 10;      // 外环每 N 次刷新打印一次；10 次约 200ms，调小更密集。
constexpr float kFallbackDiffLimit = 250.0f;     // IMU 失效时纯视觉差速限幅，主要是保护值；若 IMU 偶发断联导致猛打方向，应降低。

// 姿态串级控制参数：角度环和角速度环
constexpr float kAnglePidP = 11.0f;              // 角度环 P，将角度误差转换为目标角速度；偏航回正慢就加，来回摆就减。
constexpr float kAnglePidI = 0.0f;               // 角度环 I，通常保持 0；长时间偏航且无其他补偿时才极小幅增加，避免积分把车拉偏。
constexpr float kAnglePidD = 0.1f;               // 角度环 D，抑制角度变化过快；角度环回正过冲可加，角速度目标噪声变大就减。
constexpr float kAnglePidIntegralLimit = 100.0f; // 角度环积分限幅，当前 I 为 0 基本不起作用；启用 I 后必须配合小限幅防止积累。
constexpr float kAnglePidOutputLimit = 120.0f;   // 角度环输出目标角速度限幅，调大转向更猛；先保持不大于视觉目标角速度限幅。
constexpr float kYawTrackKp = 1.55f;             // 角度目标外环P：角度误差 -> 目标角速度，来自角速度环demo实测。
constexpr float kYawTrackToleranceDeg = 1.2f;    // 角度目标到位容差，过大会提前停车，过小会末端反复修正。
constexpr float kYawTrackSettleGyroDps = 5.0f;   // 到位时允许的残余角速度，避免车还在惯性转动就切阶段。
constexpr float kYawIntegralDeadBandDps = 1.2f;  // 角度积分死区，过滤MPU6050静止零漂。
constexpr float kYawTrackMinRateDps = 18.0f;     // 大误差末端最低角速度，克服静摩擦。
constexpr float kYawTrackFineRateDps = 12.0f;    // 小误差末端微调角速度，避免差一点不动。
constexpr float kYawTrackMinRateStartDeg = 6.0f; // 大误差最低角速度启用阈值。
constexpr float kYawTrackFineRateStartDeg = 1.2f;// 小误差微调启用阈值。
constexpr float kYawTrackDefaultMaxRateDps = 45.0f; // 角度目标默认最大角速度，90度调试时稳定可用。
constexpr float kGyroPidP = 0.165f;               // 角速度环 P，从0.22提到0.28进一步提升内环跟踪，减少GErr。
constexpr float kGyroPidI = 0.0f;                // 角速度环 I，通常保持 0；存在长期角速度误差时再从 0.001 级别试，防止积累甩尾。
constexpr float kGyroPidD = 0.0045f;              // 角速度环 D，从0.0045提到0.012增强阻尼，抑制内环振荡和超调。
constexpr float kGyroPidIntegralLimit = 50.0f;   // 角速度环积分限幅，当前 I 为 0 基本不起作用；启用 I 时先降到 5-10 再试。
constexpr float kGyroPidOutputLimit = 38.0f;     // 角速度环差速输出限幅，调大弯道能力更强但易侧滑；力度不足时提高差速输出上限。
constexpr float kGyroDeadBandDps = 1.5f;         // 陀螺仪角速度死区，小于该值认为是 0；静止数据漂移大就加，低速小修正迟钝就减。
constexpr float kGyroMeasurementLimitDps = 260.0f; // 陀螺仪测量限幅，需高于视觉目标角速度上限，否则 220dps 急弯会被内部截断成假误差。
constexpr float kGyroLpfAlpha = 0.32f;           // 陀螺仪低通滤波系数，从0.7降到0.55平衡跟踪速度和稳定性；0.7在S弯换向时GF振荡过大。
constexpr float kGyroFeedforwardGain = 0.036f;   // 目标角速度前馈系数，从0.05提到0.065增强入弯响应速度。
constexpr float kDiffOutputMaxDelta = 3.6f;      // 差速输出变化率限制，从5.0放宽到6.0让转向响应更快。
constexpr float kDiffOutputReverseMaxDelta = 13.0f; // 目标与实际角速度反向时的差速变化率，专门救 S 弯回正慢；只在车身还朝旧方向转时启用。
constexpr float kReverseGyroTargetThresholdDps = 55.0f; // 反向救车启用的目标角速度阈值，避免直道小修正误触发。
constexpr float kReverseGyroRealThresholdDps = 35.0f;   // 反向救车启用的实际角速度阈值，过滤静止零漂和小幅摆动。
constexpr float kGyroRawSpikeRejectDps = 260.0f; // 原始陀螺仪单帧尖峰过滤阈值；静止实测偶发-359dps，需在进入角速度环前拒绝。
constexpr float kGyroTargetPositiveGain = 1.03f; // 目标角速度正向补偿系数，用于修正车体正反向驱动不对称；正向欠转时略增。
constexpr float kGyroTargetNegativeGain = 1.00f; // 目标角速度负向补偿系数；负向偏大时可略降到 0.98 左右。
constexpr float kStraightHighGyroLimitDps = 120.0f; // 直行目标下异常高角速度阈值，超过后收紧差速；直道突然甩尾可降低。
constexpr float kStraightHighGyroDiffLimit = 8.0f;  // 直行异常角速度时差速限幅，调小保护更强但恢复慢；救车不足可略增。

// MPU6050 量程换算与零偏标定参数
constexpr float kMpu6050AccelScale = 16384.0f;   // 加速度量程换算，当前对应正负 2g；如果驱动量程改了必须同步改，否则加速度数值会错。
constexpr float kMpu6050GyroScale = 16.4f;       // 陀螺仪量程换算，当前对应正负 2000dps；如果驱动配置不是该量程，角速度闭环会整体失真。
constexpr int kMpu6050GyroBiasSamples = 200;     // 陀螺仪零偏采样次数，调大零偏更稳但启动更慢；零偏波动大可加到 300-500。
constexpr int kMpu6050BiasSampleDelayUs = 10000; // 零偏采样间隔，10000us 即 10ms；采样太快噪声相关性高，建议保持 5-10ms。

// 摄像头与二值化预处理参数
constexpr int kCameraWidth = 160;                // 摄像头采集宽度，调大图像更细但 CPU 压力更高；循迹优先保持 160。
constexpr int kCameraHeight = 120;               // 摄像头采集高度，调大能看更远但处理更慢；当前只取下半部分，通常保持 120。
constexpr int kCameraFps = 120;                  // 新摄像头实际只能到 120fps；按实测值配置，避免每次启动出现 FPS adjusted 警告。
constexpr lq_camera_format_t kCameraFormat = LQ_CAMERA_0CPU_MJPG; // 摄像头格式，低 CPU 占用模式适合主控；若画面异常再尝试 HIGH_MJPG。
constexpr const char *kCameraDevicePath = LQ_CAMERA_PATH; // 摄像头设备节点，默认 /dev/video0；换摄像头编号时改这里。
constexpr int kCameraManualExposure = 55;       // 手动曝光值，调大画面更亮但运动拖影可能增加；先固定曝光再看OTSU质量。
constexpr int kCameraRoiX = 0;                   // ROI 左上角 X，通常保持 0；摄像头画面横向偏移明显时先机械校正，再微调。
constexpr int kCameraRoiY = 60;                  // ROI 左上角 Y，越大越看近端、响应快但预判少；当前 ROI 高度为 60，120 高图像下这里保持最大安全值。
constexpr int kCameraRoiWidth = 160;             // ROI 宽度，应与采集宽度和压缩逻辑匹配；非必要不要改。
constexpr int kCameraRoiHeight = 60;             // ROI 高度，越大信息更多但处理慢；若远端元素判断不足可适当增大。
constexpr int kCameraColumnSampleStep = 2;       // 横向采样步长，2 表示 160 压缩到 80 列；调大更快但精度低，调小要同步修改图像数组宽度。
constexpr int kBlueHueLower = 90;                // 蓝色背景 H 下限，背景被漏检时降低，黑线被误当蓝色时提高。
constexpr int kBlueSaturationLower = 100;         // 蓝色背景 S 下限，光照弱导致蓝色饱和度低时降低，杂色误检为蓝色时提高。
constexpr int kBlueValueLower = 60;              // 蓝色背景 V 下限，暗场漏检时降低，阴影噪点误检时提高。
constexpr int kBlueHueUpper = 130;               // 蓝色背景 H 上限，背景偏紫/偏青时调整；上限过高可能吞掉非赛道区域。
constexpr int kBlueSaturationUpper = 255;        // 蓝色背景 S 上限，通常保持 255；只有特殊反光场景才收窄。
constexpr int kBlueValueUpper = 255;             // 蓝色背景 V 上限，通常保持 255；强反光误检严重时可略降。

// 图像二值化算法选择
constexpr int kImageAlgorithm = 2;               // 0=HSV蓝色背景提取，1=灰度静态阈值，2=灰度+OTSU自动阈值（当前推荐）
constexpr int kImageOtsuBlurKernel = 3;          // OTSU前灰度滤波核大小，只支持奇数；3可抑制噪点且不明显糊边。
constexpr int kImageMorphKernel = 3;             // 二值图形态学核大小，3用于去孤立点/小孔洞；过大会吞细节。

// 图像循迹与元素判断阈值
constexpr int kImageMiddleLine = 38;             // 图像压缩后中心列；本轮低速前静态实测 Det=38，先消除直道初始偏置。
constexpr int kImageTowPoint = 12;               // 前瞻行，数值越小看得越远、越提前，越大看近端、越稳；大弯出界时继续前移。
constexpr int kImageThresholdStatic = 130;       // 灰度二值化阈值；110时white_avg仍≈0.480且边界稳定，本轮提高到130验证能否压低白色比例。
constexpr int kImageThresholdDetach = 180;       // 阳光/亮斑分离阈值备用参数，强反光误判时提高，暗场丢线时降低。
constexpr int kImageScanInterval = 3;            // 普通扫边范围，调大容错更强但可能跨到错误边界；本次回到 3，减少跳到错误边线。
constexpr int kImageScanIntervalCross = 4;       // 十字/复杂元素扫边范围，调大更容易找回边但更易误补线；本次收紧，减少十字路口连转误判。
constexpr int kImageVariance = 90;               // 直道判断方差阈值，越大越容易判直道；直道不加速可增大，弯道误判直道则降低。
constexpr int kImageVarianceAcc = 45;            // 直线加速检测阈值备用参数，越大越容易进入加速；确认直线判断可靠后再放宽。
constexpr int kElementRingConfirmFrames = 2;     // 圆环入口连续确认帧数，调大更抗误判，调小更灵敏。
constexpr int kElementRingExitConfirmFrames = 3; // 圆环结束连续确认帧数，防止出环瞬间边界抖动导致状态来回跳。
constexpr int kElementCooldownFrames = 20;        // 特殊元素退出后的识别冷却帧数，避免刚回普通赛道又立刻误触发。
constexpr int kStraightVarianceAccThreshold = 12; // 静态直道 var≈9.96 已贴近阈值，略放宽到 12，避免直道在边界处反复 Normol/Straight。
constexpr int kStraightOfflineMax = 8;           // 直道允许的顶边丢线数，调大更容易判直道但可能误判弯道；丢线多时先查二值化再改。
constexpr int kStraightLeftLineMax = 3;          // 直道允许的左丢边数，调大更宽松；左边界不稳定时先调曝光和 HSV，再微调。
constexpr int kStraightRightLineMax = 3;         // 直道允许的右丢边数，调大更宽松；右边界不稳定时同样先处理图像质量。
}

#endif
