# GPIO 看门狗开机自启安装说明

## 文件说明

```text
autoboot/
├── install.sh
├── bin/
│   └── gpio_manager
├── driver/                 # 保留的屏幕驱动文件，不参与当前安装流程
└── service/
    ├── gpio-manager.service
    └── screen.service      # 保留模板，当前安装脚本会停用它
```

## 安装

```bash
scp -r autoboot/ root@192.168.x.x:/root/autoboot
ssh root@192.168.x.x
cd /root/autoboot
chmod +x install.sh
./install.sh
```

安装脚本会：

- 检查并赋予 `bin/gpio_manager` 执行权限；
- 从模板生成 `/etc/systemd/system/gpio-manager.service`；
- 启用并启动 `gpio-manager.service`；
- 停止并禁用 `screen.service`，避免旧屏幕服务与当前部署混用。

## 验证

```bash
systemctl status gpio-manager --no-pager
journalctl -u gpio-manager -f
```

`gpio_manager` 使用 FIFO 实时调度（优先级 90），监控小车进程的共享内存心跳。心跳超时后，它会直接把受保护的 PWM、方向 GPIO、舵机和蜂鸣器写入安全值。

## 启动顺序

```text
gpio_manager -> car application
```

小车程序的 `--mode=car` 默认会等待 gpio_manager 就绪；未启动看门狗时会拒绝进入正式车模模式。`--unsafe-dev-mode` 仅限悬空或台架诊断，禁止赛道使用。

## 更新 gpio_manager

替换 `bin/gpio_manager` 后执行：

```bash
systemctl restart gpio-manager
systemctl status gpio-manager --no-pager
```

## 卸载

```bash
./install.sh remove
```

卸载会停止并禁用 `gpio-manager.service`，删除对应 systemd 服务文件；不会删除当前目录。
