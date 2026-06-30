#ifndef LQ_CAR_CONFIG_HPP_
#define LQ_CAR_CONFIG_HPP_

#include "lq_camera_ex.hpp"

namespace car_config
{
// =====================================================================
// 电机硬件与最内层速度环参数
// 速度单位: m/s (车轮线速度)
// 换算: m/s = 电机RPS × (30/68) × π × 0.064 ≈ RPS × 0.08870
// 齿轮: 编码器小齿轮30齿, 车轮大齿轮68齿, 轮胎直径64mm
// =====================================================================
constexpr float kEncoderRpsToMS = 0.08870f;      // 编码器电机RPS→车轮线速度m/s换算系数
constexpr int kMotorPwmFrequencyHz = 25000;      // 电机 PWM 频率，频率过低电机会啸叫，过高会降低有效分辨率；通常保持 20k-25kHz，除非驱动器发热或响应异常。
constexpr int kMotorPwmStopDuty = 10000;         // 电机停止时的占空比
constexpr int kMotorPwmFullScale = kMotorPwmStopDuty; // 电机全速占空比数值
constexpr int kEncoderDiagPulsePwm = 2500;        // 编码器诊断测试占空比
constexpr int kEncoderDiagPulseMs = 200;          // 编码器诊断测试脉冲时间（ms）
constexpr int kMotorInitialDuty = kMotorPwmStopDuty; // 上电即写停车电平，避免初始化瞬间冲车。
constexpr int kGpioProtectHeartbeatIntervalMs = 10; // 对齐 loongson-car-system：10ms=100Hz，通过 /gpio_proj_shm 递增 heartbeat_counter。
constexpr int kWatchdogHeartbeatStallTimeoutMs = 100; // 运行期 watchdog 活性检测：heartbeat_last_ms 超过该值未更新则认为 gpio_manager 失活，主动停车退出。
constexpr float kSpeedPidP = 1800.0f;             // 【基准值】速度环比例项，实车正弦跑线测试标定。
constexpr float kSpeedPidI = 35.0f;              // 【基准值】速度环积分项，实车正弦跑线测试标定。
constexpr float kSpeedPidD = 0.0f;               // 【基准值】速度环微分项，保持0。
constexpr float kSpeedTargetAccelLimit = 1.8f;    // 轮速目标上升限幅 m/s^2，0→0.9m/s约0.5s，减少起步大阶跃。
constexpr float kSpeedTargetDecelLimit = 4.0f;    // T06弯道入弯减速太慢致丢边，从2.0提到4.0，0.6→0.44仅需39ms。
constexpr int kMotorMaxPwm = 8000;               // PWM 输出限幅，调大动力更强但更容易打滑和烧驱动；先确保满电、空载和上赛道都不发热再提高。
constexpr float kSpeedPidBrakePwm = 0.0f;        // 循迹中不主动反拖车轮；弯道放开最小正速度到 0，但不允许负轮速目标造成倒拖。
constexpr float kInnerWheelZeroTargetBrakeSpeed = 0.25f; // 禁止将负 PWM 当作内轮制动：当前硬件负PWM会反转电机
constexpr float kInnerWheelZeroTargetBrakeGain  = 0.0f;
constexpr float kInnerWheelZeroTargetBrakePwm   = 0.0f;
constexpr float kCurveSpeedGoal = 0.34f;        // 提前减速，减小弯中惯性。

// 内轮脉冲反拖制动参数
constexpr float kInnerBrakeEnableBaseSpeed = 0.25f;
constexpr float kInnerBrakeEnterSpeed = 0.55f;
constexpr float kInnerBrakeReleaseSpeed = 0.40f;
constexpr int kInnerBrakePulsePwm = 0;
constexpr int kInnerBrakePulseOnTicks = 0;
constexpr int kInnerBrakePulseOffTicks = 0;
constexpr float kStraightSpeedGoal = 0.55f;     // 低速落地验证档，先让中线跟踪稳定，再逐步加速。

// 电机起步与速度前馈参数
constexpr float kMotorStartBasePwm = 3200.0f;      // 起步克服静摩擦的基础 PWM
constexpr float kSpeedFeedforwardGain = 2000.0f;    // 目标速度前馈系数 (PWM per m/s)
constexpr float kMaxWheelSpeedGoal = 4.0f;       // 单轮目标速度上限 m/s (原100RPS×0.0887≈8.9)

// 视觉外环与差速分配参数
constexpr float kSteerKp = 2.55f;                 // 外环先恢复基准，避免修复制动后突然过猛
constexpr float kSteerKd = 1.50f;                 // 增大D项抑制振荡，从1.00提到1.50。
constexpr float kMaxDiffRatio = 1.25f;           // 恢复差速比为 1.25，避免外轮抬速过猛。
constexpr int kSteerDeadBandDefault = 2;          // T06 S弯换向死区吞2帧致出界，降到2让Raw=-2时也产生转向。
constexpr int kSteerDeadBandStraight = 2;        // 直道Raw=2~3时也参与回中，只吞掉1列内的小噪声，避免回中太慢。
constexpr int kSteerDeadBandCurve = 1;            // T06 S弯换向关键帧落在死区内，弯道死区进一步降到1。
constexpr float kMinWheelSpeedGoal = 0.0f;        // 不再给内轮强制保底速度，弯道直接采用差速分配后的目标速度。
constexpr float kOuterLoopMinPeriodSec = 0.006f;  // 对齐180Hz摄像头：0.006s≈167Hz，充分利用高帧率降低视觉延迟。
constexpr float kOuterLoopFallbackPeriodSec = 0.006f; // dt 异常时使用的兜底周期，应与外环周期一致。
constexpr float kSteerFilterAlpha = 0.70f;        // 减小视觉滤波延迟，从0.55提到0.70。
constexpr float kSteerRawJumpThreshold = 14.0f;  // Raw单帧突变判定阈值，兼容当前S弯跳边保护逻辑。
constexpr float kSteerRawMaxStepPerFrame = 7.0f; // Raw突变时每帧最大接纳步长，避免识别跳边直接反打。
constexpr float kSteerFilterMaxStepPerFrame = 5.0f; // T06 追踪弱：从3提到5，Filt每帧允许变5像素，弯道不滞后。
constexpr float kSteerFilterSignFlipMaxStepPerFrame = 1.8f; // Filt跨符号换向时更严格，避免中心附近一帧反打。
constexpr float kSteerFastFlipAlpha = 0.78f;     // S 弯大幅换向时的快速滤波系数，让 Filt 尽快跟上 Raw，避免继续朝旧方向打。
constexpr float kSteerFastFlipRawThreshold = 8.0f;  // 触发快速换向的 Raw 最小幅度，从 10 降到 8 让 S 弯更早触发快速跟踪。
constexpr float kSteerFastFlipGap = 15.0f;        // Raw 与 Filt 差值换向判定，从 18 降到 15 让中等换向也能触发。
constexpr float kSharpTurnSteerError = 4.0f;     // 提前减速，低偏即进弯道速度
constexpr float kCurveSpeedExitSteerError = 3.0f; // 弯道档退出阈值

// 单侧丢线提前预判弯道
constexpr int kCurvePreviewLossStart = 4;
constexpr int kCurvePreviewLossStop = 14;
constexpr float kCurvePreviewLossGain = 0.70f;
constexpr float kCurvePreviewErrorMax = 5.0f;

// 弯道时不能因基础速度降到 0.3 而把转向能力一起压得过低
constexpr float kCurveMinDiff = 0.52f;
constexpr float kCurveMinDiffGyroDps = 12.0f;

constexpr float kVisualGyroLimitDps = 45.0f;     // 防止继续要求底盘做不到的 -100~-120 dps 转向
constexpr float kTargetGyroMaxDeltaDps = 38.0f;  // T06 追踪弱：从26提到38，目标角速度更快爬升，弯道转向不拖沓。
constexpr float kTargetGyroSignFlipMaxDeltaDps = 210.0f; // 目标角速度换向时的变化率限制，只在正负号切换时放宽；确保大幅换向先跨过零，不继续朝旧方向打。
constexpr float kTargetGyroDeadbandReturnDeltaDps = 72.0f; // 视觉进入死区后目标角速度回零速度，调大出弯姿态收得更快，过大则直道可能有回正抽动。
constexpr bool kSteerDebugPrintEnable = true;    // 临时落地诊断：打印速度/视觉/编码器闭环状态。
constexpr int kSteerDebugPrintDivider = 30;      // 降低板端日志写入频率，保留诊断同时减少闭环抖动来源。
constexpr float kFallbackDiffLimit = 250.0f;     // IMU 失效时纯视觉差速限幅，主要是保护值
// IMU fault-tolerance parameters
constexpr int kImuMaxAllowedMissedFrames = 3;     // Maximum allowed transient IMU missed frames (3 frames = 30ms at 100Hz)
constexpr float kImuFallbackSpeedGoal = 0.40f;    // Target speed under permanent IMU failure (m/s)
constexpr int kImuRecoveryMinFrames = 5;          // Minimum stable IMU frames required before recovery

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
constexpr float kGyroPidP = 0.025f;               // 0.022 转向跟不上，略恢复 P，但仍低于此前容易顿挫的 0.030。
constexpr float kGyroPidI = 0.0f;                // 【基准值】角速度环 I，保持 0；原地阶跃测试无稳态误差无需积分。
constexpr float kGyroPidD = 0.0006f;              // D 项继续保持很弱，避免低速弯中左右轮目标抖动。
constexpr float kGyroPidIntegralLimit = 50.0f;   // 角速度环积分限幅，当前 I 为 0 基本不起作用；启用 I 时先降到 5-10 再试。
constexpr float kGyroPidOutputLimit = 65.0f;     // 角速度环差速输出限幅；78配合0.55P导致振荡，回退到65。
constexpr float kGyroDeadBandDps = 3.0f;         // T06：从5.0降到3.0，减小死区40%，保留小角速度响应。
constexpr float kGyroMeasurementLimitDps = 260.0f; // 陀螺仪测量限幅，需高于视觉目标角速度上限，否则 220dps 急弯会被内部截断成假误差。
constexpr float kGyroLpfAlpha = 0.70f;           // 减小陀螺仪滤波延迟，从0.50提到0.70。
constexpr float kGyroFeedforwardGain = 0.0055f;  // 轻微恢复前馈，弯道初段更早建立角速度。
constexpr float kDiffOutputMaxDelta = 0.10f;     // 每 5ms 差速变化限幅，降到 0.10 恢复一部分入弯响应。
constexpr float kGyroRawSpikeRejectDps = 260.0f; // 原始陀螺仪单帧尖峰过滤阈值；静止实测偶发-359dps，需在进入角速度环前拒绝。

// MPU6050 量程换算与零偏标定参数
constexpr float kMpu6050AccelScale = 16384.0f;   // 加速度量程换算，当前对应正负 2g；如果驱动量程改了必须同步改，否则加速度数值会错。
constexpr float kMpu6050GyroScale = 16.4f;       // 陀螺仪量程换算，当前对应正负 2000dps；如果驱动配置不是该量程，角速度闭环会整体失真。
constexpr int kMpu6050GyroBiasSamples = 200;     // 陀螺仪零偏采样次数，调大零偏更稳但启动更慢；零偏波动大可加到 300-500。
constexpr int kMpu6050BiasSampleDelayUs = 10000; // 零偏采样间隔，10000us 即 10ms；采样太快噪声相关性高，建议保持 5-10ms。

// =====================================================================
// 双路摄像头参数：320x240 用于 NCNN 推理，内部下采样 160x120 用于循迹
// =====================================================================
constexpr int kCameraHiresWidth = 320;           // 摄像头高分辨率采集宽度，用于 NCNN 模型推理输入。
constexpr int kCameraHiresHeight = 240;          // 摄像头高分辨率采集高度。
constexpr int kCameraWidth = 160;                // 循迹下采样宽度，保持与既有算法兼容；修改此值需同步调整 ROI 和采样步长。
constexpr int kCameraHeight = 120;               // 循迹下采样高度。
constexpr int kCameraFps = 180;                  // 摄像头支持 180fps；切换到高速模式以降低延迟、提升实时性。
constexpr lq_camera_format_t kCameraFormat = LQ_CAMERA_0CPU_MJPG; // 摄像头格式，低 CPU 占用模式适合主控；若画面异常再尝试 HIGH_MJPG。
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
constexpr float kNcnnConfidenceThreshold = 0.6f; // NCNN 推理置信度阈值，低于此值的结果视为无效。
constexpr const char *kNcnnModelParam = "tiny_classifier_fp32.ncnn.param";  // run_all.ps1 产物。
constexpr const char *kNcnnModelBin = "tiny_classifier_fp32.ncnn.bin";      // run_all.ps1 产物。
constexpr int kNcnnObjectBottomGap = 5;         // ROI 底边与红色标记顶边的间隔（与 --object-bottom-gap 5 一致）。
// 标签列表（与 ImageFolder 子目录名一致，按字母排序）
constexpr const char *kNcnnLabel0 = "supplies";
constexpr const char *kNcnnLabel1 = "vehicles";
constexpr const char *kNcnnLabel2 = "weapons";
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
constexpr int kElementRingConfirmFrames = 1;     // 圆环入口连续确认帧数；十字口误入左环时先加大确认。
constexpr int kElementRingExitConfirmFrames = 4; // 圆环结束连续确认帧数，防止出环瞬间边界抖动导致状态来回跳。
constexpr int kElementCooldownFrames = 45;        // 特殊元素退出后的识别冷却帧数，避免刚回普通赛道又立刻误触发。
constexpr int kElementRingExitStraightHoldFrames = 14; // 圆环结束后短暂锁直行，让出环姿态先收稳。
constexpr int kElementRingMinActiveFrames = 55;   // 入环后最少保持约1.1s，再允许进入结束判定，避免刚拐进环就被出口条件拉直。
constexpr int kRingEntryCenterBias = 30;          // 圆环进环阶段中线向环心侧额外偏置，帮助车头真正切入环岛；不进环先增大。
constexpr int kRingInnerCenterBias = 18;          // 圆环内阶段中线向环心侧额外偏置；过小会持续骑外侧，过大容易内切。
constexpr int kLeftRingRightLineBlockMax = 12;   // 左圆环入口允许的右侧丢线上限；左环入口左侧缺线为主，对侧少量异常不直接拒绝。
constexpr int kLeftRingOfflineBlockMax = 18;     // 左圆环入口允许的顶端丢线上限。
constexpr int kLeftRingMinLeftLost = 5;          // 左圆环入口最小左侧丢线数；调小更容易进左环，调大更抗普通左弯误判。
constexpr int kLeftRingMaxPoint2Y = 42;          // 左圆环第二特征点最高允许位置；调大可放宽左环识别。
constexpr int kRightRingLeftLineBlockMax = 18;   // 右圆环入口允许的左侧丢线上限；右环入口伴随左侧少量异常时仍允许确认。
constexpr int kRightRingOfflineBlockMax = 25;    // 右圆环入口允许的顶端丢线上限；U型入口远端缺线较多，过低会漏检。
constexpr int kRightRingMinRightLost = 6;        // 右圆环入口最小右侧丢线数；调小更容易进右环，调大更抗普通右弯误判。
constexpr int kRightRingMinPoint2Y = 15;         // 右圆环第二特征点最低位置；调小可提前识别右环，调大更保守。
constexpr int kRingLossOnlyMinLost = 18;         // 无几何特征时的单侧丢线兜底门槛；提高以过滤十字/U弯单侧丢线。
constexpr int kRingLossOnlyMaxEntryError = 30;   // 单侧丢线兜底只允许在车身尚未大幅偏离中线时入环，避免十字急弯误触发。
constexpr float kRingSpeedGoal = 0.53f;           // 环岛内速度上限 m/s (原6RPS×0.0887≈0.53)
constexpr float kRingForcedSteerError = 18.0f;   // 环岛已锁定后，视觉回到中线/死区时仍保持最小转向量，防止在环内走直。
constexpr int kRingForcedSteerReleaseFlag = 7;   // 到环内/出环阶段后释放强制转向，避免一直同向绕圈。
constexpr float kRingExitSteerError = -10.0f;    // 左环进入flag>=7后给反向出环转向；右环自动取相反号。
constexpr float kRingEntryGyroLimitDps = 64.0f;  // 入环前段目标角速度上限，避免入口一把打死冲出边界。
constexpr float kRingSceneGyroLimitDps = 62.0f;  // 已锁定环岛场景后的视觉目标角速度上限，防止环内长期贴边。
constexpr float kRingEntryMaxDiff = 15.0f;       // 入环前段最大差速，压住外轮过快导致的入口甩出。
constexpr float kRingMaxDiff = 13.0f;            // 环岛内最大差速；绕圈说明同向角速度太强，先压低一点帮助出环。
constexpr int kBadVisionOfflineThreshold = 55;    // 关了曲线补偿后弯道自然丢行多，从45提到55避免弯道误触发bad_vision。
constexpr int kBadVisionCenterHoldOfflineThreshold = 55; // 十字/近乎全丢线时不沿用旧偏差，先按中线低速通过。

// 十字路口角点补线参数
constexpr int kCrossMinWhiteLine = 8;             // 触发十字角点检测的最小双丢边行数；过小会误触发普通弯道。
constexpr int kCrossCornerScanStartRow = 10;      // 角点搜索起始行（从近端开始）。
constexpr int kCrossCornerScanEndRow = 50;        // 角点搜索终止行（远端截止）。
constexpr int kCrossCornerMinBorderLen = 5;       // 角点检测需要的连续有边行数最小值；过小会把噪点当角点。
constexpr int kCrossRepairSlopeLimit = 2.0f;      // 补线斜率绝对值上限，防止补线角度过大飞掉。
constexpr int kCrossMinCornerYGap = 5;            // 上下角点之间最小行距，过滤太近的伪角点。
constexpr int kCrossRepairOfflineExtend = 3;      // 补线后OFFLine向上扩展行数，确保GetDet能看到补好的线。
constexpr float kBadVisionSpeedGoal = 0.0f;        // 近乎全丢线时停止；看不到中线时不继续按旧误差冲出界。
constexpr float kBadVisionGyroLimitDps = 38.0f;   // 弯道出口偏差30+时22dps太弱追不回来，提到38配合Kp=2.40。
constexpr float kBadVisionMaxDiff = 0.75f;        // 图像不稳/单边丢线时的差速上限，介于0.65止摆和0.90回中之间。
constexpr int kStraightVarianceAccThreshold = 12; // 静态直道 var≈9.96 已贴近阈值，略放宽到 12，避免直道在边界处反复 Normol/Straight。
constexpr int kStraightOfflineMax = 8;           // 直道允许的顶边丢线数，调大更容易判直道但可能误判弯道；丢线多时先查二值化再改。
constexpr int kStraightLeftLineMax = 3;          // 直道允许的左丢边数，调大更宽松；左边界不稳定时先调曝光和 HSV，再微调。
constexpr int kStraightRightLineMax = 3;         // 直道允许的右丢边数，调大更宽松；右边界不稳定时同样先处理图像质量。
constexpr int kSingleSideLossThreshold = 42;      // T06单侧丢边经常49-52且触发img_warn，先提高门槛减少普通弯误触发单边追踪。

// 稳定中心线重构：按边界可信度做补线，避免单侧丢线/宽度异常时中线乱飘。
constexpr bool kRaceUdpDebugEnable = false;       // 正式 car 模式关闭 JPEG 图传；图传只放到 vision-debug/低速诊断模式。
constexpr bool kBasicLineBaselineMode = true;     // 基础循迹基线：无元素模式先跳过十字补线和曲线跨帧补偿。
constexpr bool kCurveCenterCompEnable = true;     // 开启轻量级可信度补线；基础基线模式会临时跳过。
constexpr int kCurveCenterLossStart = 4;          // 单侧丢线达到该值后提高单边重构权重；过小会误伤普通小弯，过大则大弯介入偏晚。
constexpr int kCurveCenterOppositeLossMax = 12;   // 对侧也严重丢线时少信单侧边界，改走本帧行间预测，避免双边弱观测乱补。
constexpr int kCurveCenterWidthTolerance = 16;    // 宽度偏离期望赛道宽度超过该值时，即使标记为T也按不可靠边界处理。
constexpr int kCurveCenterSingleSideBias = 0;     // 当前过弯偏内切，先取消单边补线向弯内的额外偏置。
constexpr int kCurveCenterMaxRowStep = 5;         // TFT显示/控制中线相邻行最大跳变，抑制大弯远端中线突然冲出边界。
constexpr int kCurveCenterFrameMaxStep = 4;       // 同一行跨帧最大接纳步长，吸收抖动但允许真实弯道连续移动。
constexpr int kCurveCenterBadFrameMaxStep = 1;    // 双边都弱或宽度明显异常时跨帧更保守，避免坏图像把控制目标带飞。
constexpr int kCurveCenterMinValidWidth = 10;     // 小于该宽度视为不可用，防止边界夹到一起后仍参与平均。
constexpr int kCurveCenterMaxValidWidth = 72;     // 大于该宽度视为不可用，防止扫到画面边缘造成中线跳边。
// =====================================================================
// Speed Loop Tuning & Telemetry Diagnostics (VOFA)
// =====================================================================
constexpr int kEncoderLineCount = 512;            // 编码器线数（如512线）
constexpr uint32_t kEncoderStalePeriodThreshold = 50000000U; // 编码器周期超时阈值（160MHz下约0.31s，即停止）
constexpr char kSpeedTuneUdpHost[] = "192.168.1.100"; // UDP 调试目标主机 IP
constexpr int kSpeedTuneUdpPort = 8081;          // UDP 调试目标端口
constexpr int kSpeedTuneControlPeriodMs = 10;     // 速度环调试控制周期（10ms = 100Hz）
constexpr int kSpeedTuneArmMaxMs = 5000;         // 等待起跑最大超时（5秒）
constexpr int kSpeedTuneArmStableMs = 500;        // 起跑前静止稳定时间要求（500ms）
constexpr int kSpeedTuneEncoderInvalidHoldMs = 1500; // 编码器无效容忍时间；启动后捕获周期约需1.3s恢复有效。
constexpr int kSpeedTuneActiveMs = 1200;         // 调试正向行驶总时间（1200ms）
constexpr int kSpeedTuneCooldownMs = 1000;       // 停止后冷却等待时间（1000ms）
constexpr float kSpeedTuneDistanceLimitM = 1.20f; // 调试正向最大行驶位移限值（1.2米）
constexpr int kSpeedTuneEncoderProbeMs = 500;     // 编码器极性探测阶段持续时间（500ms）
constexpr int kSpeedTuneEncoderProbePwm = 2500;   // 编码器极性探测阶段测试PWM
constexpr float kSpeedTuneHardMaxSpeed = 2.00f;   // 调试过程物理安全限速（2.0m/s）
constexpr int kSpeedTuneLaunchMs = 120;          // 起步动力补偿脉冲时间（120ms）
constexpr int kSpeedTuneLaunchPwm = 5000;         // 起步动力补偿脉冲PWM；实测4000左轮不动，5000两轮可可靠起转。
constexpr int kSpeedTuneLowMotionHoldMs = 500;    // 行驶中低速异常持续保护时间（500ms）
constexpr int kSpeedTuneMaxPwm = 5000;            // 调试过程PWM上限；仅用于speed-tune，正式车速环仍使用kMotorMaxPwm。
constexpr float kSpeedTuneMinForwardDistanceForReturnM = 0.30f; // 触发自动返回所需的最小正向行驶位移（0.3m）
constexpr int kSpeedTuneMismatchHoldMs = 150;     // 左右轮偏差异常持续保护时间（150ms）
constexpr float kSpeedTuneMotionStartThreshold = 0.06f; // 运动起步检测速度阈值；架空静止编码器残余约0.035m/s，放宽到0.06避免误判未静止。
constexpr float kSpeedTuneOverspeedStop = 1.20f;  // 超速保护阈值（1.20m/s）
constexpr float kSpeedTuneReturnStopDistanceM = 0.10f; // 返回停止保护提前量（0.1m）
constexpr int kSpeedTuneReturnTimeoutMs = 3000;   // 自动返回过程最大超时（3000ms）
constexpr int kSpeedTuneReverseBrakePwm = 1000;   // 倒车或刹车时的测试辅助PWM
constexpr int kSpeedTuneSaturationHoldMs = 200;   // 占空比饱和异常持续保护时间（200ms）
constexpr int kSpeedTuneSettleMaxMs = 1000;       // 减速静止等待的最大超时（1000ms）
constexpr int kSpeedTuneSettleStableMs = 300;     // 判定完全静止所需的稳定时间（300ms）
constexpr float kSpeedTuneSineMaxSpeed = 0.55f;   // 正弦扫频测试最大目标速度（0.55m/s）
constexpr float kSpeedTuneSineMinSpeed = 0.20f;   // 正弦扫频测试最小目标速度（0.20m/s）
constexpr float kSpeedTuneSinePeriodSec = 1.00f;  // 正弦扫频测试的扫频周期（1.0s）
constexpr int kSpeedTuneStallHoldMs = 300;        // 电机堵转异常持续保护时间（300ms）
constexpr float kSpeedTuneStallSpeedThreshold = 0.05f; // 电机堵转判定的速度阈值（0.05m/s）
constexpr float kSpeedTuneWheelMismatchStop = 0.25f; // 左右车轮速度偏差保护阈值（0.25m/s）
}

#endif
