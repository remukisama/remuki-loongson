# 小车算法优化与详细调参顺序

本文面向当前项目代码，重点覆盖控制算法、视觉算法、循迹算法、元素判断，以及 `user_app/include/car_config.hpp` 中集中参数的调参建议。

## 当前算法链路

当前主控逻辑大致可以理解成 4 层：

```text
摄像头图像
  -> HSV 二值化和 ROI 压缩
  -> 图像扫线、边界、中线、元素判断
  -> 视觉偏差转换为目标角速度
  -> MPU6050 角速度闭环生成左右轮差速
  -> 左右轮速度 PID
  -> PWM 和方向 GPIO 输出
```

核心文件：

- `user_app/src/image_cv.cpp`：摄像头采集、翻转、ROI、HSV 蓝色背景识别、二值化压缩到 `img1[60][80]`。
- `user_app/src/image.cpp`：扫线、边界、中线、直道/元素状态判断。
- `user_app/src/my_motor.cpp`：速度目标、视觉外环、IMU 角速度闭环、左右轮速度环、PWM 输出。
- `user_app/src/my_cascade_pid.cpp`：角度环和角速度环串级 PID。
- `user_app/include/car_config.hpp`：当前建议优先修改的集中参数。

## 控制算法现状

### 速度控制

当前左右轮是最内层速度环：

```text
目标轮速 Speed_Goal_l/r
  - 编码器速度 speed_encoder_L/R
  -> 增量式 PI
  -> speed_out_l/r
  -> motor_Duty_L/R
```

主要参数：

- `kSpeedPidP`
- `kSpeedPidI`
- `kSpeedPidD`
- `kMotorMaxPwm`
- `kCurveSpeedGoal`
- `kStraightSpeedGoal`
- `kMaxWheelSpeedGoal`

优化建议：

- 先确认编码器方向和电机方向一致。目标为正时，编码器速度也应为正。
- 速度环先单独调，暂时不要叠加视觉和陀螺仪。
- 建议增加速度环日志：目标速度、实际速度、误差、PWM 输出。
- 建议加入目标速度斜坡，避免目标速度从 0 突然跳到 15 或更高。
- 建议给左右轮分别保留 PID 参数，因为左右电机、轮胎、编码器安装通常不完全一致。

### 方向控制

当前方向控制是：

```text
视觉偏差 steer_error
  -> 低通滤波
  -> 视觉 PD
  -> 目标角速度 target_gyro_z
  -> MPU6050 角速度闭环
  -> 差速 diff_speed
  -> 左右轮目标速度
```

主要参数：

- `kSteerKp`
- `kSteerKd`
- `kMaxDiffRatio`
- `kSteerFilterAlpha`
- `kTargetGyroMaxDeltaDps`
- `kGyroPidP`
- `kGyroPidD`
- `kGyroPidOutputLimit`
- `kDiffOutputMaxDelta`

优化建议：

- 先把 IMU 角速度环调稳定，再加视觉外环。
- 视觉外环不要一开始就很大，过大的 `kSteerKp` 会让车头追线过猛。
- `kSteerKd` 只在 P 能基本进弯后再加，用来压制过冲和 S 弯摆动。
- `kTargetGyroMaxDeltaDps` 和 `kDiffOutputMaxDelta` 是保护小车不抽动的关键参数，调大前先确认图像偏差没有跳变。
- 如果小车弯道内切，优先降低 `kMaxDiffRatio` 或提高 `kMinWheelSpeedGoal`，不要只降低速度。

## 视觉算法现状

当前图像流程：

1. 摄像头采集 `160x120`。
2. 图像翻转 180 度。
3. 裁剪 ROI，默认取下半部分 `160x60`。
4. 转 HSV。
5. 提取蓝色背景范围。
6. 反相得到二值图。
7. 高斯滤波降噪。
8. 横向每 2 列采样一次，压缩成 `60x80`。

主要参数：

- `kCameraWidth`
- `kCameraHeight`
- `kCameraFps`
- `kCameraManualExposure`
- `kCameraRoiY`
- `kCameraRoiHeight`
- `kBlueHueLower`
- `kBlueHueUpper`
- `kBlueSaturationLower`
- `kBlueValueLower`

优化建议：

- 固定曝光后再调 HSV，不要自动曝光和 HSV 同时变。
- 建议做一份图传调试画面，同时显示原图、HSV mask、压缩后的二值图、中线和边界。
- HSV 阈值建议现场采样：晴天、阴影、反光、暗场各保存几帧，再取稳定范围。
- 当前先高斯滤波再采样，若边缘变糊导致边界漂移，可以尝试改为形态学开/闭操作。
- 如果速度提高，应适当降低 `kCameraRoiY`，让车看得更远；如果误判远端元素，应提高 `kCameraRoiY`。

## 循迹算法现状

当前循迹依赖 `image.cpp` 中的扫线结果：

- 从底部开始寻找左右边界。
- 根据上一行边界和扫描区间继续向上搜索。
- 计算每行中心、宽度、丢边、断线等状态。
- 根据前瞻行和中线计算偏差。

主要参数：

- `kImageMiddleLine`
- `kImageTowPoint`
- `kImageScanInterval`
- `kImageScanIntervalCross`
- `kStraightVarianceAccThreshold`
- `kStraightOfflineMax`
- `kStraightLeftLineMax`
- `kStraightRightLineMax`

优化建议：

- 前瞻行不要固定到底，建议后续做动态前瞻：速度越快，看得越远。
- 中线偏差建议使用多行加权，而不是只依赖单一前瞻点，能减小局部噪点影响。
- 丢边时应输出“可信度”，低可信度时降低车速并减小方向输出。
- 弯道中可以用近端行保证不出界，远端行用于提前转向，两者混合比随速度变化。
- 十字、环岛、坡道等元素状态中，普通循迹输出应该被状态机接管，而不是所有逻辑同时抢控制权。

## 元素判断现状

当前 `image.cpp` 中已经有道路类型、圆环、十字、车库等状态变量，例如：

- `Road_type`
- `ImageFlag.image_element_rings`
- `ImageFlag.image_element_rings_flag`
- `Barn_in`
- `Straight`
- `Cross`
- `LeftCirque`
- `RightCirque`

优化建议：

- 每个元素使用明确状态机：未识别、连续确认、进入、处理中、退出、冷却。
- 元素确认必须加入连续帧计数，避免单帧误识别。
- 元素进入和退出建议同时使用图像特征、编码器距离、IMU 角速度或角度积分。
- 元素处理期间应临时修改速度、前瞻、死区和最大差速，不要只依赖通用循迹参数。
- 每个元素都要有冷却距离，刚退出圆环或十字时不要立刻再次识别。

## 关键参数调参建议

### 摄像头与二值化

| 参数 | 现值 | 调大效果 | 调小效果 | 优先调参建议 |
| --- | --- | --- | --- | --- |
| `kCameraManualExposure` | 100 | 画面更亮，可能拖影和反光增加 | 画面更暗，可能蓝底漏检 | 先固定曝光，再调 HSV |
| `kCameraRoiY` | 60 | 看近端，响应直接但预判少 | 看远端，提前入弯但噪声多 | 低速用 60，高速可试 50 |
| `kBlueHueLower` | 90 | 蓝色范围变窄，误检少 | 蓝色范围变宽，漏检少 | 蓝底漏检时降低 |
| `kBlueHueUpper` | 130 | 蓝色范围变宽，漏检少 | 蓝色范围变窄，误检少 | 非蓝色被吞掉时降低 |
| `kBlueSaturationLower` | 60 | 排除低饱和噪声 | 暗场蓝底更容易识别 | 阴影漏检时降低 |
| `kBlueValueLower` | 60 | 排除暗噪声 | 暗场更宽容 | 光照暗时降低 |

### 循迹

| 参数 | 现值 | 调大效果 | 调小效果 | 优先调参建议 |
| --- | --- | --- | --- | --- |
| `kImageMiddleLine` | 39 | 认为中心向右 | 认为中心向左 | 只用于补偿摄像头微小偏装 |
| `kImageTowPoint` | 24 | 看近端，车更稳 | 看远端，提前转向 | 高速逐步调小，抖动则调大 |
| `kImageScanInterval` | 3 | 更容易找回边界，但可能误搜 | 更严格，但易丢边 | 弯道丢边时加 1 |
| `kImageScanIntervalCross` | 6 | 十字容错更强，但易误判 | 更严格，但可能漏十字 | 十字漏检加大，误检减小 |
| `kStraightVarianceAccThreshold` | 5 | 更容易判直道 | 更难判直道 | 直道不触发可逐步加到 8-12 |

### 视觉方向外环

| 参数 | 现值 | 调大效果 | 调小效果 | 优先调参建议 |
| --- | --- | --- | --- | --- |
| `kSteerKp` | 10.2 | 转向更积极，可能摆头 | 更稳但进弯慢 | 先调 P，能过弯但不过冲 |
| `kSteerKd` | 10.2 | 抑制过冲，可能放大噪声 | 更平顺但压不住摆动 | P 合适后再调 D |
| `kSteerFilterAlpha` | 0.35 | 更跟手但更抖 | 更稳但延迟 | 图像跳变先减小 |
| `kMaxDiffRatio` | 1.2 | 急弯能力更强，易甩尾 | 更稳但可能转不过 | 急弯不足再加 |
| `kTargetGyroMaxDeltaDps` | 10.0 | 目标切换快，可能抽动 | 更顺滑，入弯慢 | S 弯突刺先减小 |

### 陀螺仪角速度环

| 参数 | 现值 | 调大效果 | 调小效果 | 优先调参建议 |
| --- | --- | --- | --- | --- |
| `kGyroPidP` | 0.12 | 角速度跟随更强，差速冲击更大 | 更柔和但跟随慢 | 每次加减 0.02 |
| `kGyroPidD` | 0.006 | 抑制甩动，可能放大噪声 | 更平顺但阻尼弱 | 甩尾加，咔咔响减 |
| `kGyroPidOutputLimit` | 30 | 最大差速更强 | 限制更保守 | 急弯不足才加 |
| `kGyroLpfAlpha` | 0.25 | 角速度更灵敏但更抖 | 更稳但延迟 | IMU 噪声大先减小 |
| `kDiffOutputMaxDelta` | 4.0 | 差速变化更快 | 差速更平滑 | 抽动时减小 |

### 速度环

| 参数 | 现值 | 调大效果 | 调小效果 | 优先调参建议 |
| --- | --- | --- | --- | --- |
| `kSpeedPidP` | 57.0 | 轮速响应更快，易过冲 | 更稳但跟随慢 | 空载先调到不过冲 |
| `kSpeedPidI` | 26.5 | 消除稳态误差，易积累冲击 | 稳态误差变大 | 车速跟不上再加 |
| `kMotorMaxPwm` | 8000 | 动力上限更高 | 动力更保守 | 发热、打滑时降低 |
| `kCurveSpeedGoal` | 15 | 弯道更快，风险高 | 更稳 | 所有弯稳定后再加 |
| `kStraightSpeedGoal` | 15 | 直道更快 | 更稳 | 最后阶段再加 |

## 详细调参顺序

### 第 0 步：机械和电气检查

目标：排除所有“不是算法”的问题。

检查项：

- 左右轮方向一致。
- 电机正转时编码器读数为正。
- 左右轮空转阻力接近。
- 电池电压稳定，低电压会让 PID 参数全部失真。
- 摄像头固定牢靠，画面中心接近车体中心。
- MPU6050 固定牢靠，静止时角速度接近 0。

不通过这一关，不要调算法。

### 第 1 步：摄像头和二值化

目标：让 `img1[60][80]` 稳定表示赛道。

调参顺序：

1. 固定 `kCameraManualExposure`。
2. 调 `kBlueHueLower`、`kBlueHueUpper`，让蓝色背景稳定被识别。
3. 调 `kBlueSaturationLower`、`kBlueValueLower`，适配暗光和阴影。
4. 调 `kCameraRoiY`，决定看近端还是远端。
5. 确认压缩后的 80 列图像没有大面积噪声。

判定标准：

- 直道时左右边界稳定。
- 弯道时边界不会突然断掉。
- 阴影和反光不会导致整片区域翻转。

### 第 2 步：循迹输出检查

目标：电机不开或低速时，确认视觉偏差方向正确。

检查项：

- 黑线在图像右侧时，`steer_error` 的方向应让小车向右修正。
- 黑线在图像左侧时，方向相反。
- `ImageStatus.Det_True` 不应单帧大幅跳变。
- 直道上 `Road_type` 不应频繁在直道和普通状态之间抖动。

优先调：

1. `kImageMiddleLine`
2. `kImageTowPoint`
3. `kImageScanInterval`
4. `kSteerFilterAlpha`

### 第 3 步：单独调左右轮速度环

目标：不看图像、不进弯，仅让左右轮速度稳定跟随目标。

建议做法：

- 架起小车，让轮子悬空。
- 给左右轮相同目标速度。
- 打印目标速度、实际速度、PWM 输出。

调参顺序：

1. `kSpeedPidI = 0`，先调 `kSpeedPidP`。
2. P 调到响应快但不过冲。
3. 加 `kSpeedPidI`，让稳态误差变小。
4. 如果编码器噪声很小，再考虑 `kSpeedPidD`。
5. 最后设置 `kMotorMaxPwm`，避免输出过大。

判定标准：

- 左右轮同目标时速度接近。
- 目标变化时无明显长时间振荡。
- 停车目标为 0 时 PWM 能回到 0。

### 第 4 步：低速纯循迹

目标：先用低速跑完整基础赛道，不追求速度。

建议参数：

- `kCurveSpeedGoal` 保持较低。
- `kStraightSpeedGoal` 暂时等于 `kCurveSpeedGoal`。
- `kMaxDiffRatio` 不要太大。

调参顺序：

1. 调 `kSteerKp`，让小车能进入弯道。
2. 如果来回摆，降低 `kSteerKp` 或增大 `kSteerFilterAlpha` 的平滑程度。
3. 如果出弯过冲，再加 `kSteerKd`。
4. 如果急弯转不过，再略加 `kMaxDiffRatio` 或降低 `kMinWheelSpeedGoal`。

判定标准：

- 直道不蛇形。
- 普通弯不内切。
- 出弯不反向大幅修正。

### 第 5 步：加入陀螺仪角速度闭环

目标：让转向不只靠视觉，还能用 IMU 抑制甩尾。

调参顺序：

1. 确认 `gyro_z` 静止零偏正常。
2. 调 `kGyroDeadBandDps`，静止时输出不乱跳。
3. 小幅调 `kGyroPidP`，让实际角速度能跟随目标角速度。
4. 用 `kGyroPidD` 抑制过冲。
5. 用 `kGyroPidOutputLimit` 限制最大差速。
6. 用 `kDiffOutputMaxDelta` 抑制电机突然抽动。

判定标准：

- 弯中车尾不明显甩。
- S 弯切换不抽动。
- 目标角速度和实际角速度同向且延迟可接受。

### 第 6 步：提高速度

目标：在稳定基础上逐步提速。

调参顺序：

1. 每次只提高 `kStraightSpeedGoal` 或 `kCurveSpeedGoal` 1-2。
2. 先提高直道速度，再提高弯道速度。
3. 速度提高后重新检查 `kImageTowPoint`，高速通常需要看远一点。
4. 若高速入弯晚，减小 `kImageTowPoint` 或 `kCameraRoiY`。
5. 若高速直道蛇形，增大 `kSteerDeadBandStraight` 或降低 `kSteerKp`。

判定标准：

- 提速后不会破坏低速时的稳定性。
- 失误类型可重复，而不是随机丢线。

### 第 7 步：元素逐个调

目标：每次只打开一个元素逻辑。

建议顺序：

1. 十字。
2. 左圆环。
3. 右圆环。
4. 车库。
5. 坡道或其他特殊元素。

每个元素都按下面流程：

1. 只跑该元素相关赛道。
2. 记录进入前 20 帧、处理中、退出后 20 帧的图像状态。
3. 增加连续确认帧数，先防误判。
4. 增加编码器距离或 IMU 角度条件，防止同一元素重复进入。
5. 元素处理期间单独设置速度和差速限幅。
6. 退出后设置冷却距离。

判定标准：

- 不误入。
- 不漏入。
- 进入后状态不会来回跳。
- 退出后能平滑回到普通循迹。

### 第 8 步：整车回归测试

目标：验证所有参数组合在一起仍然稳定。

建议记录：

- `ImageStatus.Det_True`
- `ImageStatus.Road_type`
- `Speed_Goal_l/r`
- `speed_encoder_L/R`
- `dbg_steer_error`
- `dbg_target_gyro_z`
- `dbg_diff_speed`
- `gyro_z`

测试顺序：

1. 满电低速 3 圈。
2. 满电目标速度 3 圈。
3. 电压下降后 3 圈。
4. 强光/阴影条件下 3 圈。
5. 每次只改一个参数，记录修改前后的表现。

## 常见现象和优先处理

### 直道蛇形

优先检查：

1. 图像中心线是否稳定。
2. `kSteerKp` 是否过大。
3. `kSteerDeadBandStraight` 是否过小。
4. `kSteerFilterAlpha` 是否过大。

建议改法：

- 降低 `kSteerKp`。
- 增大 `kSteerDeadBandStraight`。
- 降低 `kSteerFilterAlpha`。
- 确认左右轮速度环没有一边快一边慢。

### 入弯太晚

优先检查：

1. `kImageTowPoint` 是否看得太近。
2. `kCameraRoiY` 是否太大。
3. `kSteerKp` 是否太小。
4. `kTargetGyroMaxDeltaDps` 是否限制太强。

建议改法：

- 减小 `kImageTowPoint`。
- 减小 `kCameraRoiY`。
- 小幅增加 `kSteerKp`。
- 小幅增加 `kTargetGyroMaxDeltaDps`。

### 弯道内切

优先检查：

1. 速度是否过高。
2. `kMaxDiffRatio` 是否过大。
3. `kMinWheelSpeedGoal` 是否太低，导致内轮反转太多。
4. 出弯时 `kSteerKd` 是否不足。

建议改法：

- 降低 `kCurveSpeedGoal`。
- 降低 `kMaxDiffRatio`。
- 提高 `kMinWheelSpeedGoal`，例如从 `-12` 改到 `-5` 或 `0`。
- 适当增加 `kSteerKd`。

### 电机咔咔响或突然抽动

优先检查：

1. 图像偏差是否单帧跳变。
2. `kTargetGyroMaxDeltaDps` 是否过大。
3. `kDiffOutputMaxDelta` 是否过大。
4. `kGyroPidD` 是否放大噪声。

建议改法：

- 降低 `kSteerFilterAlpha`。
- 降低 `kTargetGyroMaxDeltaDps`。
- 降低 `kDiffOutputMaxDelta`。
- 降低 `kGyroPidD`。

### 丢线或误识别元素

优先检查：

1. 原图是否曝光过度或过暗。
2. HSV mask 是否稳定。
3. `kImageScanInterval` 是否太小。
4. 元素判断是否缺少连续帧确认。

建议改法：

- 先调曝光和 HSV。
- 再调 `kImageScanInterval`。
- 元素状态加入连续帧和冷却距离。

## 推荐的最终调参原则

- 一次只改一个参数。
- 每次改动幅度小一点，记录现象。
- 先低速稳定，再逐步提速。
- 先普通循迹，再元素判断。
- 先图像质量，再控制参数。
- 控制参数不要补偿机械问题，机械问题先修机械。
- 如果现象随机，优先查图像、供电、接线和线程周期，而不是盲目调 PID。
