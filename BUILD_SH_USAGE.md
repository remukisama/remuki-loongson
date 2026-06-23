# build.sh 使用说明

本文说明当前 `main/build.sh` 的用法，以及远程传输时目标 IP 应该在哪里填写。

## 脚本位置

脚本文件：

```bash
main/build.sh
```

你可以在项目根目录运行：

```bash
bash main/build.sh
```

也可以进入 `main/` 后运行：

```bash
cd main
./build.sh
```

如果提示没有执行权限，先执行：

```bash
chmod +x main/build.sh
```

## 运行前目录要求

脚本默认按下面的相对目录查找工具链和依赖：

```text
Loongson_2K300_301_LIB/
├── main/
├── tools/
│   ├── loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6/
│   └── LQ_Dep_libs/
│       ├── opencv_install/
│       ├── ncnn_install/
│       └── ffmpeg_install/
```

如果工具链目录不存在，脚本会尝试在 `tools/` 下查找对应 `.tar.xz` 压缩包并解压。

如果 `opencv_install`、`ncnn_install`、`ffmpeg_install` 不存在，脚本会尝试在 `tools/LQ_Dep_libs/` 下查找对应 `.tar.xz` 压缩包并解压。

## 仅编译

不传 IP 时，只编译，不传输到开发板：

```bash
bash main/build.sh
```

或：

```bash
cd main
./build.sh
```

编译成功后，可执行文件位置为：

```text
main/build/main
```

## 编译并传输到开发板

目标 IP 不需要写进脚本文件里，直接作为第一个参数传给 `build.sh`。

例如你的开发板 IP 是 `192.168.1.100`：

```bash
bash main/build.sh 192.168.1.100
```

或：

```bash
cd main
./build.sh 192.168.1.100
```

脚本会执行：

1. 检查工具链和依赖库。
2. 生成 `toolchain_path.cmake`。
3. 删除旧的 `main/build/`。
4. 重新 CMake 配置并编译。
5. 通过 `scp` 把 `main/build/main` 传输到开发板。

## 编译、传输并远程运行

第二个参数传 `-r`，远程会自动执行 `main --mode=car`：

```bash
bash main/build.sh 192.168.1.100 -r
```

脚本会在传输完成后通过 SSH 在开发板后台启动 `main --mode=car`。

如果需要进入调参模式：

```bash
APP_ARGS="--mode=tune" bash main/build.sh 192.168.1.100 -r
```

## 增量编译与全量清理

脚本默认使用增量编译（已有 build 目录时直接 `make -j`），不会每次都删除重建。

如果需要全量清理后重新编译：

```bash
bash main/build.sh --clean
```

如果需要强制重新执行 CMake 配置（例如修改了 CMakeLists.txt）：

```bash
bash main/build.sh --reconfigure
```

## 目标 IP 写在哪里

推荐做法：不要写进文件，直接写在命令第一个参数里。

```bash
bash main/build.sh 你的开发板IP
```

例如：

```bash
bash main/build.sh 192.168.1.100
```

当前脚本中相关代码在 `main/build.sh` 的参数解析位置：

```bash
BOARD_IP="${1:-}"
RUN_FLAG="${2:-}"
```

含义是：

- `$1`：你运行脚本时写的第一个参数，也就是开发板 IP。
- `$2`：你运行脚本时写的第二个参数，目前用于 `-r`。

如果你一定想写一个默认 IP，可以把：

```bash
BOARD_IP="${1:-}"
```

改成：

```bash
BOARD_IP="${1:-192.168.1.100}"
```

这样不传参数时也会默认传输到 `192.168.1.100`。不过更推荐命令行传 IP，避免换网络后误传到旧地址。

## 远程用户名、路径和日志

当前默认远程用户名：

```bash
BOARD_USER="root"
```

当前默认远程目标路径：

```bash
BOARD_TARGET_PATH="${BOARD_TARGET_PATH:-~}"
```

也就是说默认传到开发板 root 用户的家目录。可以临时指定远程目录：

```bash
BOARD_TARGET_PATH=/home/root bash main/build.sh 192.168.1.100
```

远程运行日志默认文件名：

```bash
BOARD_LOG_FILE="${BOARD_LOG_FILE:-${EXECUTABLE_NAME}.log}"
```

可以临时指定：

```bash
BOARD_LOG_FILE=main.log bash main/build.sh 192.168.1.100 -r
```

## 常用命令速查

仅编译：

```bash
bash main/build.sh
```

编译并传输：

```bash
bash main/build.sh 192.168.1.100
```

编译、传输并运行：

```bash
bash main/build.sh 192.168.1.100 -r
```

指定远程目录后传输：

```bash
BOARD_TARGET_PATH=/home/root bash main/build.sh 192.168.1.100
```

指定远程目录并运行：

```bash
BOARD_TARGET_PATH=/home/root BOARD_LOG_FILE=main.log bash main/build.sh 192.168.1.100 -r
```

查看远程程序是否运行：

```bash
ssh root@192.168.1.100 'pgrep -f main'
```

停止远程程序：

```bash
ssh root@192.168.1.100 'pkill -f main'
```

查看远程日志：

```bash
ssh root@192.168.1.100 'cat main.log'
```

如果你使用了 `BOARD_LOG_FILE=/home/root/main.log` 这种绝对路径，就按你设置的路径查看。

## 注意事项

- 开发板和电脑要在同一网络，且开发板 SSH 服务可用。
- 第一次连接开发板时，SSH 可能要求确认 host key，按提示输入 `yes`。
- 如果 `scp` 失败，优先检查 IP、用户名、开发板网络、目标路径权限。
- 当前脚本会检测 `pkg-config` 和 `cmake`，缺失时会尝试通过 `sudo apt` 安装；如果在离线环境使用，建议提前手动安装。
- `main/toolchain_path.cmake` 是脚本生成文件，里面使用相对工具链路径，不需要手动维护。
