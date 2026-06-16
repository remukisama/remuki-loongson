# 开机自启安装说明

## 文件说明

```
自启程序/
├── install.sh              一键安装/卸载脚本
├── bin/
│   ├── gpio_manager         GPIO 看门狗守护进程
│   └── screen_app           屏幕线程
├── driver/
│   ├── TFT18_dri.ko         TFT18 屏幕内核驱动
│   └── TFT18_dev.ko         TFT18 屏幕设备节点
└── service/
    ├── gpio-manager.service  看门狗 systemd 服务（模板）
    └── screen.service        屏幕 systemd 服务（模板）
```

## 安装步骤

### 1. 将整个文件夹传到板子任意位置

```bash
scp -r 自启程序/ root@192.168.x.x:/root/autoboot
```

### 2. SSH 登录板子，执行安装

```bash
cd /root/autoboot
chmod +x install.sh
./install.sh
```

**文件夹不会被移动**，脚本只是：
- 设置 bin/ 下文件的执行权限
- 将 service 模板中的路径替换为当前目录的实际路径，写入 `/etc/systemd/system/`
- 启用开机自启 + 立即启动

### 3. 验证

```bash
systemctl status gpio-manager
systemctl status screen
```

## 启动顺序

```
1. gpio_manager    ← 最先启动（实时调度 FIFO，优先级 90）
2. screen_app      ← 第二启动（自动 insmod TFT18 内核模块）
3. car_project     ← 小车工程（手动启动或自行配置服务）
```

## 卸载

```bash
./install.sh remove
```

停止服务、禁用开机自启、删除 systemd 服务文件、卸载内核模块。**本文件夹不会被删除。**

## 注意事项

- 安装后看门狗和屏幕会**立即启动**，不需要重启
- 看门狗崩溃会自动重启（`Restart=always`）
- 屏幕依赖看门狗（`Requires=gpio-manager.service`）
- **不要移动或删除此文件夹**，systemd 服务直接索引这里的文件
- 更新编译产物后，覆盖 bin/ 里的文件，执行 `systemctl restart gpio-manager screen` 即可
