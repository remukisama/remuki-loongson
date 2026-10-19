# 每个环节具体调参方法

本文是一份按实车调试顺序写的操作手册。核心原则是：先让每个单环节稳定，再把环节串起来；一次只改一个参数；每次改完必须记录现象。

主要参数文件：

```text
user_app/include/car_config.hpp
```

建议每次调参都记录：

```text
日期：
电池电压：
赛道环境：
改动参数：
改动前：
改动后：
现象：
是否保留：
```

## 0. 调参前准备

### 0.1 固定测试条件

调参前先做这些事：

1. 电池充满或保持同一电压区间。
2. 轮胎、底盘、摄像头、IMU 固定牢靠。
3. 左右电机接线不松。
4. 赛道光照尽量稳定。
5. 每次只改一个参数。

如果电池电压明显下降，上一轮参数可能全部不准。

### 0.2 必须打印或观察的量

建议调参时尽量输出这些变量：

```cpp
ImageStatus.Det_True
ImageStatus.MiddleLine
ImageStatus.Road_type
Speed_Goal_l
Speed_Goal_r
speed_encoder_L
speed_encoder_R
speed_out_l
speed_out_r
dbg_steer_error
dbg_target_gyro_z
dbg_diff_speed
imu_data.gyro_z
```

如果没有上位机，至少用串口或 SSH 打印关键量。调参不能只靠肉眼看车。

## 1. 机械和方向校准

这一环节不改算法参数，先确认硬件方向正确。

### 1.1 电机方向

测试方法：

1. 架空小车。
2. 单独给左轮一个正 PWM。
3. 看左轮是否向前转。
4. 单独给右轮一个正 PWM。
5. 看右轮是否向前转。

合格标准：

- 左右轮正 PWM 都让车向前。
- 左右轮反 PWM 都让车向后。

如果不合格：

- 优先改电机方向 GPIO 逻辑或接线。
- 不要靠 PID 参数补偿方向错误。

### 1.2 编码器方向

测试方法：

1. 架空小车。
2. 让左轮正转。
3. 打印 `speed_encoder_L`。
4. 让右轮正转。
5. 打印 `speed_encoder_R`。

合格标准：

- 正转时编码器值为正。
- 反转时编码器值为负。
- 左右轮同样 PWM 时，速度数量级接近。

如果正转读数为负：

- 改编码器 A/B 相顺序，或在编码器读取函数里反号。
- 不建议在速度 PID 里硬凑。

### 1.3 摄像头中心

测试方法：

1. 把小车放在直道正中。
2. 打开图传或打印 `ImageStatus.Det_True`。
3. 观察识别到的中线是否接近 `kImageMiddleLine`。

参数：

```cpp
kImageMiddleLine
```

调法：

- 如果车在赛道正中，但识别偏差长期偏左或偏右 2 个像素以内，可微调 `kImageMiddleLine`。
- 如果偏差很大，先调摄像头安装位置，不要先改参数。

建议步长：

```text
每次加减 1
```

进入下一步标准：

- 小车正对直道时，视觉偏差接近 0。

## 2. 摄像头曝光调参

曝光决定图像稳定性，必须先调曝光再调 HSV。

参数：

```cpp
kCameraManualExposure
```

### 2.1 调试方法

1. 打开图传 Demo 或能看到原图的调试程序。
2. 让小车分别对准直道、弯道、阴影、反光区域。
3. 固定摄像头位置。
4. 从当前值 `100` 开始，每次改 `10-20`。

### 2.2 怎么判断曝光过高

现象：

- 画面发白。
- 蓝色背景变浅。
- 反光区域大面积被误识别。
- 车跑快时边缘拖影明显。

处理：

```text
降低 kCameraManualExposure
```

建议：

```text
100 -> 80 -> 60
```

### 2.3 怎么判断曝光过低

现象：

- 蓝色背景发黑。
- HSV mask 中蓝色背景断断续续。
- 阴影处容易丢线。

处理：

```text
提高 kCameraManualExposure
```

建议：

```text
100 -> 120 -> 140
```

### 2.4 进入下一步标准

- 原图中蓝色背景和黑线肉眼可明显分开。
- 阴影区不大面积丢失。
- 反光区不大面积炸白。

## 3. HSV 二值化调参

当前视觉用 HSV 提取蓝色背景，然后反相得到黑线/非蓝色区域。

参数：

```cpp
kBlueHueLower
kBlueHueUpper
kBlueSaturationLower
kBlueValueLower
kBlueSaturationUpper
kBlueValueUpper
```

通常优先调前 4 个：

```cpp
kBlueHueLower
kBlueHueUpper
kBlueSaturationLower
kBlueValueLower
```

### 3.1 先调 H 范围

H 决定颜色类型。

当前：

```cpp
kBlueHueLower = 90
kBlueHueUpper = 130
```

调试方法：

1. 固定曝光。
2. 看 HSV mask。
3. 目标是蓝色背景完整变成蓝色区域，黑线不要被识别为蓝色。

如果蓝色背景漏检：

```text
降低 kBlueHueLower 或提高 kBlueHueUpper
```

建议步长：

```text
每次 5
```

示例：

```cpp
kBlueHueLower: 90 -> 85
kBlueHueUpper: 130 -> 135
```

如果非蓝色区域也被识别成蓝色：

```text
提高 kBlueHueLower 或降低 kBlueHueUpper
```

示例：

```cpp
kBlueHueLower: 90 -> 95
kBlueHueUpper: 130 -> 125
```

### 3.2 再调 S 下限

S 是饱和度。

当前：

```cpp
kBlueSaturationLower = 60
```

如果阴影里蓝色背景漏检：

```text
降低 kBlueSaturationLower
```

建议：

```text
60 -> 50 -> 40
```

如果灰色、白色、反光区域误识别为蓝色：

```text
提高 kBlueSaturationLower
```

建议：

```text
60 -> 70 -> 80
```

### 3.3 最后调 V 下限

V 是亮度。

当前：

```cpp
kBlueValueLower = 60
```

如果暗处蓝色背景漏检：

```text
降低 kBlueValueLower
```

如果暗噪声被误识别：

```text
提高 kBlueValueLower
```

建议步长：

```text
每次 10
```

### 3.4 进入下一步标准

在直道、弯道、阴影、反光区域都满足：

- 蓝色背景连续。
- 黑线区域稳定。
- mask 不大面积闪烁。
- 车静止时，`ImageStatus.Det_True` 不乱跳。

## 4. ROI 和前瞻调参

ROI 决定看图像哪一部分，前瞻决定用哪一行或哪一区域作为转向依据。

参数：

```cpp
kCameraRoiY
kCameraRoiHeight
kImageTowPoint
```

### 4.1 调 `kCameraRoiY`

当前：

```cpp
kCameraRoiY = 60
```

含义：

- 越大：看得越近，车更稳，但入弯晚。
- 越小：看得越远，入弯提前，但容易被远端噪声影响。

调试方法：

1. 低速跑直道和普通弯。
2. 如果入弯明显晚，减小 `kCameraRoiY`。
3. 如果直道因远端噪声蛇形，提高 `kCameraRoiY`。

建议步长：

```text
每次 5
```

示例：

```cpp
60 -> 55 -> 50
```

或：

```cpp
60 -> 65
```

### 4.2 调 `kImageTowPoint`

当前：

```cpp
kImageTowPoint = 24
```

含义：

- 数值越小：看更远。
- 数值越大：看更近。

调试方法：

1. 低速稳定后再调。
2. 如果入弯晚，减小。
3. 如果直道抖，增大。
4. 如果出弯反打严重，先增大一点试试。

建议步长：

```text
每次 2
```

示例：

```cpp
24 -> 22 -> 20
```

或：

```cpp
24 -> 26
```

进入下一步标准：

- 直道偏差平稳。
- 普通弯能提前进入。
- 出弯不会大幅反向修正。

## 5. 扫线参数调参

扫线参数决定边界搜索是否稳定。

参数：

```cpp
kImageScanInterval
kImageScanIntervalCross
```

### 5.1 调普通扫线范围

当前：

```cpp
kImageScanInterval = 3
```

如果弯道经常丢边：

```text
增大 kImageScanInterval
```

建议：

```text
3 -> 4 -> 5
```

如果边界经常跳到错误位置：

```text
减小 kImageScanInterval
```

建议：

```text
3 -> 2
```

判断方法：

- 打印或图传显示左右边界。
- 看边界是否连续，不要只看小车是否跑起来。

### 5.2 调十字/复杂区域扫线范围

当前：

```cpp
kImageScanIntervalCross = 6
```

如果十字漏检或复杂元素边界找不回来：

```text
增大 kImageScanIntervalCross
```

如果十字误检、乱补线：

```text
减小 kImageScanIntervalCross
```

建议步长：

```text
每次 1
```

进入下一步标准：

- 普通弯边界连续。
- 十字区域不乱跳。
- 没有因为扫线范围太大跨到错误边界。

## 6. 直道判断参数调参

直道判断会影响速度和死区。

参数：

```cpp
kStraightVarianceAccThreshold
kStraightOfflineMax
kStraightLeftLineMax
kStraightRightLineMax
kSteerDeadBandStraight
kSteerDeadBandCurve
```

### 6.1 先确认直道判断是否稳定

观察：

```cpp
ImageStatus.straight_acc_flag
ImageStatus.Road_type
```

直道上：

```text
straight_acc_flag 应该稳定为 1
```

弯道上：

```text
straight_acc_flag 应该为 0
```

### 6.2 直道识别太严格

现象：

- 明明在直道，却很少进入直道状态。
- `straight_acc_flag` 经常为 0。

调法：

```text
增大 kStraightVarianceAccThreshold
增大 kStraightOfflineMax
增大 kStraightLeftLineMax / kStraightRightLineMax
```

建议顺序：

1. 先调 `kStraightVarianceAccThreshold`。
2. 再调 `kStraightOfflineMax`。
3. 最后调左右丢边阈值。

建议步长：

```text
kStraightVarianceAccThreshold 每次 +2
kStraightOfflineMax 每次 +1
kStraightLeftLineMax / kStraightRightLineMax 每次 +1
```

### 6.3 直道识别太宽松

现象：

- 弯道也被判直道。
- 入弯时死区变大，车不转。
- 直道加速提前进入弯道导致冲出去。

调法：

```text
减小 kStraightVarianceAccThreshold
减小 kStraightOfflineMax
减小 kStraightLeftLineMax / kStraightRightLineMax
```

建议步长同上。

### 6.4 调直道/弯道死区

当前：

```cpp
kSteerDeadBandStraight = 3
kSteerDeadBandCurve = 2
```

直道蛇形：

```text
增大 kSteerDeadBandStraight
```

建议：

```text
3 -> 4
```

弯道反应慢：

```text
减小 kSteerDeadBandCurve
```

建议：

```text
2 -> 1
```

弯中抖动：

```text
增大 kSteerDeadBandCurve
```

建议：

```text
2 -> 3
```

进入下一步标准：

- 直道状态稳定。
- 弯道不会被误判为直道。
- 死区切换后车身动作没有突然变懒或突然抽动。

## 7. 速度环 PID 调参

这是最内层，必须在方向环之前调好。

参数：

```cpp
kSpeedPidP
kSpeedPidI
kSpeedPidD
kMotorMaxPwm
```

### 7.1 测试方式

推荐架空小车调。

步骤：

1. 暂时不跑视觉循迹。
2. 左右轮给同一个目标速度。
3. 打印目标速度、实际速度、PWM。
4. 从低目标速度开始，例如 10。

观察：

```text
目标速度 Speed_Goal_l/r
实际速度 speed_encoder_L/R
输出 PWM speed_out_l/r
```

### 7.2 先调 P

先临时让：

```cpp
kSpeedPidI = 0
kSpeedPidD = 0
```

调 `kSpeedPidP`。

如果响应慢：

```text
增大 kSpeedPidP
```

如果速度来回振荡：

```text
减小 kSpeedPidP
```

建议步长：

```text
每次 5
```

合格标准：

- 给定目标后，轮速能较快接近目标。
- 不出现持续振荡。

### 7.3 再调 I

恢复或逐步增加：

```cpp
kSpeedPidI
```

如果轮速长期低于目标：

```text
增大 kSpeedPidI
```

如果停车后还顶电机，或速度越调越冲：

```text
减小 kSpeedPidI
```

建议步长：

```text
每次 2-3
```

合格标准：

- 稳态误差小。
- 停车目标为 0 时输出能回 0。

### 7.4 最后才考虑 D

当前：

```cpp
kSpeedPidD = 0.0f
```

只有编码器速度很干净，而且速度变化过冲明显时才加 D。

建议：

```text
0 -> 0.5 -> 1.0
```

如果编码器速度跳动明显，不要加 D。

### 7.5 调 PWM 限幅

参数：

```cpp
kMotorMaxPwm
```

如果电机明显动力不足：

```text
增大 kMotorMaxPwm
```

如果发热、打滑、冲击大：

```text
减小 kMotorMaxPwm
```

建议步长：

```text
每次 500
```

进入下一步标准：

- 左右轮速度都能稳定跟随。
- 同目标时左右轮速度差不大。
- 不发热、不抖、不持续振荡。

## 8. 低速视觉外环调参

参数：

```cpp
kSteerKp
kSteerKd
kSteerFilterAlpha
kMaxDiffRatio
kMinWheelSpeedGoal
```

先让速度低一点：

```cpp
kCurveSpeedGoal = 低速
kStraightSpeedGoal = 与弯道速度相同
```

### 8.1 先调 P

暂时把 D 影响降到较小：

```text
保持 kSteerKd 不大，或先用当前值观察
```

如果小车入弯不够、转不过：

```text
增大 kSteerKp
```

建议步长：

```text
每次 1.0
```

如果直道蛇形、弯道过冲：

```text
减小 kSteerKp
```

合格标准：

- 普通弯能过。
- 不严重蛇形。
- 不明显内切。

### 8.2 再调 D

如果 P 已经能进弯，但出弯或 S 弯摆头：

```text
增大 kSteerKd
```

建议步长：

```text
每次 1.0
```

如果电机咔咔响、方向输出变噪：

```text
减小 kSteerKd
```

判断方法：

- 看 `dbg_target_gyro_z` 是否出现尖峰。
- 如果目标角速度跳得很大，D 可能太大或视觉偏差太噪。

### 8.3 调视觉滤波

参数：

```cpp
kSteerFilterAlpha
```

当前：

```cpp
0.35
```

如果图像偏差噪声大、车头小幅抖：

```text
减小 kSteerFilterAlpha
```

建议：

```text
0.35 -> 0.30 -> 0.25
```

如果入弯明显迟钝：

```text
增大 kSteerFilterAlpha
```

建议：

```text
0.35 -> 0.40 -> 0.45
```

### 8.4 调最大差速比例

参数：

```cpp
kMaxDiffRatio
```

如果急弯转不过：

```text
增大 kMaxDiffRatio
```

建议：

```text
1.2 -> 1.3 -> 1.4
```

如果弯道内切、甩尾：

```text
减小 kMaxDiffRatio
```

建议：

```text
1.2 -> 1.1 -> 1.0
```

### 8.5 调内轮最低速度

参数：

```cpp
kMinWheelSpeedGoal
```

当前：

```cpp
-12
```

如果内轮反转导致甩尾：

```text
提高 kMinWheelSpeedGoal
```

建议：

```text
-12 -> -8 -> -5 -> 0
```

如果急弯明显转不过：

```text
降低 kMinWheelSpeedGoal
```

建议：

```text
-12 -> -15
```

进入下一步标准：

- 低速能跑完整普通赛道。
- 直道不蛇形。
- 弯道不明显内切。

## 9. 陀螺仪角速度环调参

参数：

```cpp
kGyroDeadBandDps
kGyroLpfAlpha
kGyroPidP
kGyroPidD
kGyroPidOutputLimit
kDiffOutputMaxDelta
kTargetGyroMaxDeltaDps
```

### 9.1 先看静止噪声

小车静止，打印：

```cpp
imu_data.gyro_z
```

如果静止时 `gyro_z` 在正负 1.5 以内跳：

```text
kGyroDeadBandDps = 1.5 基本合适
```

如果静止时跳到正负 3：

```text
提高 kGyroDeadBandDps
```

建议：

```text
1.5 -> 2.0 -> 3.0
```

如果小角速度响应迟钝：

```text
降低 kGyroDeadBandDps
```

### 9.2 调陀螺仪滤波

参数：

```cpp
kGyroLpfAlpha
```

如果 `gyro_z` 跳动明显：

```text
减小 kGyroLpfAlpha
```

建议：

```text
0.25 -> 0.20 -> 0.15
```

如果角速度响应慢：

```text
增大 kGyroLpfAlpha
```

建议：

```text
0.25 -> 0.30
```

### 9.3 调角速度环 P

观察：

```cpp
dbg_target_gyro_z
imu_data.gyro_z
dbg_diff_speed
```

如果目标角速度有了，但实际角速度跟不上：

```text
增大 kGyroPidP
```

建议：

```text
0.12 -> 0.14 -> 0.16
```

如果差速输出太冲、车身摆动：

```text
减小 kGyroPidP
```

建议：

```text
0.12 -> 0.10 -> 0.08
```

### 9.4 调角速度环 D

如果实际角速度过冲、车尾甩：

```text
增大 kGyroPidD
```

建议：

```text
0.006 -> 0.008 -> 0.010
```

如果电机抖、角速度噪声被放大：

```text
减小 kGyroPidD
```

建议：

```text
0.006 -> 0.004 -> 0.002
```

### 9.5 调输出限幅和变化率

最大差速输出：

```cpp
kGyroPidOutputLimit
```

如果急弯不够：

```text
增大 kGyroPidOutputLimit
```

如果弯中甩尾：

```text
减小 kGyroPidOutputLimit
```

差速变化率：

```cpp
kDiffOutputMaxDelta
```

如果方向突然抽动：

```text
减小 kDiffOutputMaxDelta
```

建议：

```text
4.0 -> 3.0 -> 2.0
```

如果转向太慢：

```text
增大 kDiffOutputMaxDelta
```

建议：

```text
4.0 -> 5.0
```

目标角速度变化率：

```cpp
kTargetGyroMaxDeltaDps
```

如果 S 弯目标切换太猛：

```text
减小 kTargetGyroMaxDeltaDps
```

如果入弯太慢：

```text
增大 kTargetGyroMaxDeltaDps
```

进入下一步标准：

- `dbg_target_gyro_z` 和 `gyro_z` 方向一致。
- 实际角速度能跟随目标。
- 差速输出不尖峰。
- 车身转向不甩尾。

## 10. 速度提升调参

参数：

```cpp
kCurveSpeedGoal
kStraightSpeedGoal
kSharpTurnSteerError
kSteerDeadBandStraight
kImageTowPoint
kCameraRoiY
```

### 10.1 先提直道速度

调法：

```text
kStraightSpeedGoal 每次 +1 或 +2
```

例如：

```text
15 -> 17 -> 19
```

如果直道蛇形：

1. 增大 `kSteerDeadBandStraight`。
2. 降低 `kSteerKp`。
3. 减小 `kSteerFilterAlpha`。
4. 检查左右速度环是否一致。

### 10.2 再提弯道速度

调法：

```text
kCurveSpeedGoal 每次 +1
```

如果弯道内切：

1. 降低 `kCurveSpeedGoal`。
2. 减小 `kMaxDiffRatio`。
3. 提高 `kMinWheelSpeedGoal`。
4. 增大 `kSteerKd`。

如果弯道转不过：

1. 增大 `kSteerKp`。
2. 减小 `kImageTowPoint`。
3. 减小 `kCameraRoiY`。
4. 增大 `kMaxDiffRatio`。

### 10.3 调急弯降速判定

参数：

```cpp
kSharpTurnSteerError
```

如果急弯速度降得太晚：

```text
降低 kSharpTurnSteerError
```

建议：

```text
50 -> 45 -> 40
```

如果普通直道或小弯经常被当成急弯：

```text
提高 kSharpTurnSteerError
```

建议：

```text
50 -> 55 -> 60
```

进入下一步标准：

- 提速后仍能稳定跑完整普通赛道。
- 失误位置固定且可解释。
- 没有随机丢线或随机甩尾。

## 11. 元素判断调参

元素包括十字、圆环、车库、坡道等。元素调参一定要逐个打开，不要一起调。

### 11.1 每个元素都按这个流程

1. 关闭其他元素判断。
2. 单独跑目标元素。
3. 记录进入前、进入中、退出后的图像。
4. 先调识别，不调控制。
5. 再调处理过程。
6. 最后调退出和冷却。

### 11.2 元素识别参数怎么调

如果误识别：

```text
提高确认条件
增加连续帧确认
增加编码器距离约束
增加进入角度或位置约束
```

如果漏识别：

```text
放宽图像条件
降低连续帧要求
扩大扫线范围
检查二值化是否丢特征
```

建议每个元素至少有这些状态：

```text
未识别
疑似
确认进入
处理中
准备退出
已退出
冷却
```

### 11.3 十字调参

常见现象：

- 十字漏检。
- 普通弯误判十字。
- 十字中间补线乱跳。

优先调：

```cpp
kImageScanIntervalCross
kImageTowPoint
kCameraRoiY
```

调法：

- 十字漏检：增大 `kImageScanIntervalCross`。
- 普通弯误判十字：减小 `kImageScanIntervalCross`，增加连续帧确认。
- 十字中间乱跳：元素处理中临时降低速度，使用固定补线策略。

### 11.4 圆环调参

圆环建议不要只靠单帧图像判断。

建议组合：

```text
图像特征 + 连续帧 + 编码器距离 + IMU 角度变化
```

常见现象：

- 入环太早：增加连续帧确认或提高图像特征阈值。
- 入环太晚：放宽图像特征或减小确认帧数。
- 环内甩尾：降低 `kCurveSpeedGoal`，减小 `kMaxDiffRatio`，提高 `kMinWheelSpeedGoal`。
- 出环后反打：增加出环冷却距离，出环后短时间降低 `kSteerKp`。

### 11.5 车库调参

车库通常要更保守。

建议：

- 车库识别必须连续确认。
- 停车距离用编码器控制，不要只靠图像。
- 进入车库后降低速度和最大差速。

常见现象：

- 提前停车：增加停车编码器距离。
- 过冲：降低车库速度，提前触发减速。
- 误判车库：增加连续帧确认和冷却距离。

## 12. 整体调参顺序总表

严格按下面顺序调：

```text
1. 电机方向
2. 编码器方向
3. 摄像头中心
4. 曝光
5. HSV
6. ROI
7. 扫线
8. 直道判断
9. 左右轮速度环
10. 低速视觉外环
11. 陀螺仪角速度环
12. 普通赛道低速闭环
13. 提高直道速度
14. 提高弯道速度
15. 十字
16. 左圆环
17. 右圆环
18. 车库
19. 全元素综合测试
20. 不同电压和光照回归测试
```

每进入下一步前，上一环节必须稳定。否则后面参数会越调越乱。

## 13. 现象到参数的快速索引

### 直道蛇形

优先顺序：

```text
kSteerKp 降低
kSteerDeadBandStraight 增大
kSteerFilterAlpha 降低
kSpeedPidP 检查是否过大
左右轮速度一致性检查
```

### 入弯太晚

优先顺序：

```text
kImageTowPoint 减小
kCameraRoiY 减小
kSteerKp 增大
kTargetGyroMaxDeltaDps 增大
kCurveSpeedGoal 降低
```

### 弯道内切

优先顺序：

```text
kCurveSpeedGoal 降低
kMaxDiffRatio 降低
kMinWheelSpeedGoal 提高
kSteerKd 增大
kImageTowPoint 增大一点
```

### 急弯转不过

优先顺序：

```text
kSteerKp 增大
kMaxDiffRatio 增大
kMinWheelSpeedGoal 降低
kImageTowPoint 减小
kCameraRoiY 减小
```

### 电机抽动

优先顺序：

```text
kTargetGyroMaxDeltaDps 降低
kDiffOutputMaxDelta 降低
kSteerFilterAlpha 降低
kSteerKd 降低
kGyroPidD 降低
```

### 图像丢线

优先顺序：

```text
kCameraManualExposure
kBlueHueLower / kBlueHueUpper
kBlueSaturationLower
kBlueValueLower
kImageScanInterval
```

### 元素误判

优先顺序：

```text
增加连续帧确认
增加编码器距离条件
增加冷却距离
收紧图像特征阈值
降低元素处理速度
```

## 14. 推荐单次调参记录模板

```text
参数：
原值：
新值：
测试位置：
电池电压：
速度目标：
现象变化：
是否解决问题：
是否引入新问题：
结论：
```

示例：

```text
参数：kSteerKp
原值：10.2
新值：11.2
测试位置：普通左弯
电池电压：12.1V
速度目标：15
现象变化：入弯提前，但出弯有轻微反打
是否解决问题：解决入弯太晚
是否引入新问题：轻微过冲
结论：保留 11.2，下一步增加 kSteerKd
```

## 15. 最重要的经验

- 图像不稳，不要调 PID。
- 编码器方向错，不要调速度环。
- 速度环没稳，不要调方向环。
- 普通循迹没稳，不要调元素。
- 低速没稳，不要提速。
- 每次只改一个参数，改完记录。
