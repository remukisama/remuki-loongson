# UDP 实时调参

本方案不需要改动 5 ms 控制线程，也不需要复制粘贴终端日志。

## 1. 上位机：先启动接收与分析

在仓库根目录执行：

```bash
python3 tools/tuning/udp_tuning_live.py --bind 0.0.0.0 --port 24405
```

上位机防火墙需要允许 UDP `24405` 入站。接收器会把每次测试保存为：

```text
udp_tuning_logs/<session>_T05.log
udp_tuning_logs/<session>_T11.log
udp_tuning_logs/<session>_T06.log
```

## 2. 板端：将原程序终端输出实时转发

把 `<PC_IP>` 改为上位机的局域网 IPv4 地址：

```bash
stdbuf -oL -eL ./main --mode=tune 2>&1 | \
python3 tools/tuning/udp_tuning_forwarder.py --host <PC_IP> --port 24405
```

`stdbuf` 使 C++ 的 `printf` 按行刷新；T05 的键盘输入仍由原终端接收，不受管道影响。

## 3. 实时输出含义

- `T05_LIVE` / `T05_FINAL`：速度环候选。只有出现 `DIRECT` 时，才能复制 `kSpeedPidP`、`kSpeedPidI`。
- `T11_LIVE` / `T11_FINAL`：角速度环候选。只有出现 `DIRECT` 时，才能复制 gyro 参数。
- `T06_LIVE` / `T06_FINAL`：弯道闭环判定。`DIRECT-GATED` 需要先满足其打印的前置条件；其他 `HOLD` 表示禁止改 PID。

T05 首次实时分析可能仍处于最小日志时长之前，最终以 `T05_FINAL` 为准。

## 4. 推荐流程

```text
T05 0.20~0.60 m/s
→ T05 0.45~0.90 m/s
→ T11
→ T06 低速普通弯
→ T07 提速
```

每轮只接受一个测试的一个候选改动。电池、赛道、轮胎、相机高度和光照发生变化后，必须重新采集对比日志。
