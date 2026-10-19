# Loongson 2K300/301 项目优化建议

调研日期：2026-04-29

## 项目概览

当前目录是一个面向龙芯 2K300/301 智能车的 C/C++ 工程，主要包含：

- `main/`：用户程序入口、CMake 构建脚本、构建/清理脚本。
- `user_app/`：小车控制、图像处理、编码器、电机、IMU、PID 等业务代码。
- `libraries/`：通用类型、外设寄存器封装、摄像头、UDP/CANFD/I2C 等库。
- `driver/`：多个 Linux 内核驱动模块及对应发送脚本。
- `example/`：外设和调参 Demo。
- `tools/`：交叉编译工具链和依赖库相关文件。

整体功能覆盖面比较完整，但仓库里混合了源码、构建产物、工具链、调试 Demo 和当前比赛/实验代码。优先建议先解决“会影响构建和主控运行”的问题，再做目录治理和接口收敛。

## 优先级建议

### P0：优先修复，可能直接影响构建或运行

1. `main/build.sh` 参数解析和脚本路径存在明显问题

   位置：

   - `main/build.sh:40`
   - `main/build.sh:455`
   - `main/build.sh:417`

   问题：

   - 第 40 行使用了 `{BASE_SOURCES[0]}`，第 455 行使用了 `${BASE_SOURCES[0]}`，这里大概率是把 Bash 内置的 `${BASH_SOURCE[0]}` 写错了。
   - 第 417 行把 `BOARD_IP` 固定成 `192.168.251.100`，导致脚本注释里写的“仅编译：`./build.sh`”不会真正仅编译，而是默认尝试连接和传输到这个 IP。

   建议改法：

   ```bash
   SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
   PARENT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
   BOARD_IP="${1:-}"
   RUN_FLAG="${2:-}"
   cd "${SCRIPT_DIR}"
   ```

   同时建议在脚本前部增加：

   ```bash
   set -euo pipefail
   ```

   加了 `set -u` 后，所有可选参数都要使用 `${1:-}`、`${2:-}` 这种写法。

2. `main/main.cpp` 当前主控代码不可达

   位置：

   - `main/main.cpp:55`
   - `main/main.cpp:56`
   - `main/main.cpp:57-169`

   问题：

   - 当前程序会执行 `lq_gyro_loop_verify_demo();` 后立刻 `return 0;`。
   - 后面的摄像头初始化、电机初始化、信号处理、控制线程、主循环全部不可达。

   建议改法：

   - 不建议通过频繁注释/取消注释来切换 Demo。
   - 增加运行模式配置，例如 `APP_MODE_DEMO_GYRO_VERIFY`、`APP_MODE_CAR_MAIN_LOOP`。
   - 或者用 CMake 选项控制入口：

   ```cmake
   option(LQ_RUN_DEMO "Run selected demo instead of car main loop" OFF)
   add_compile_definitions(LQ_SELECTED_DEMO=0)
   ```

   然后在 `main.cpp` 中集中判断，避免主控流程被临时 Demo 截断。

3. `main/main.cpp` 中线程先 `detach` 后 `join`，恢复主循环后会出问题

   位置：

   - `main/main.cpp:117`
   - `main/main.cpp:118`
   - `main/main.cpp:167`

   问题：

   - `threadA.detach()` 后线程已经与 `std::thread` 对象分离。
   - 后面再调用 `threadA.join()` 时，线程对象已经不可 join。

   建议改法：

   - 如果主程序退出时需要等待控制线程安全结束：删除 `detach()`，保留 `join()`。
   - 如果控制线程完全后台运行：保留 `detach()`，删除 `join()`，但要保证退出时有可靠的停机流程。
   - 对智能车控制，更建议不 `detach`，用 `exit_flag` 通知退出后 `join()`，确保电机停机逻辑能收尾。

4. `user_app/include/motor.hpp` 与实际实现变量名不一致

   位置：

   - `user_app/include/motor.hpp:12-13`
   - `user_app/src/my_motor.cpp:60`
   - `user_app/src/my_motor.cpp:67`
   - `user_app/src/my_motor.cpp:412-413`

   问题：

   - 头文件声明的是 `speed_encoder_R`、`speed_encoder_L`。
   - 实际实现里曾经有一组拼写错误的编码器变量名，和头文件声明不一致。
   - 部分 Demo 曾经直接 `extern` 实现文件里的变量，说明项目里绕开了头文件，长期容易出现链接错误或调试变量混乱。

   建议改法：

   - 统一改成 `speed_encoder_left`、`speed_encoder_right`。
   - 头文件只暴露确实需要跨模块读取的状态。
   - 调试 Demo 不要手写 `extern`，统一包含头文件。

5. `MyMpu6050::init()` 设备 ID 不匹配时仍返回成功

   位置：

   - `user_app/src/my_mpu6050.cpp:22-45`

   问题：

   - 读取 ID 不是 `0x68` 时打印失败信息，但最后仍然 `return true;`。
   - 上层会误以为 IMU 初始化成功，闭环控制可能拿到无效数据。

   建议改法：

   ```cpp
   if (id != 0x68) {
       std::cerr << "MPU6050 Init Failed..." << std::endl;
       return false;
   }
   ```

   如果确实存在“部分硬件 ID 读不准但数据可读”的场景，建议增加显式配置项，例如 `allow_id_mismatch`，不要默认成功。

### P1：建议尽快优化，能明显提升可维护性

1. 增加根目录 `README.md` 和 `.gitignore`

   现状：

   - 根目录没有 `README.md`。
   - 根目录没有 `.gitignore`。
   - 当前存在 `main/build/`、`main/build_backup/`、`main/build_intellisense/`、驱动 `build/`、`.o`、`.ko`、`.cmd`、`Module.symvers`、`modules.order` 等构建产物。
   - `tools/` 目录约 446 MB，包含交叉工具链，适合独立下载或压缩包管理，不适合长期混在源码仓库里。

   建议新增 `.gitignore` 内容：

   ```gitignore
   main/build*/
   main/.cache/
   driver/**/build/
   driver/**/.tmp_versions/
   driver/**/*.o
   driver/**/*.ko
   driver/**/*.mod.c
   driver/**/*.mod.o
   driver/**/*.cmd
   driver/**/Module.symvers
   driver/**/modules.order
   tools/
   main/toolchain_path.cmake
   ```

   `README.md` 建议至少写清楚：

   - 支持平台与硬件连接。
   - 首次构建步骤。
   - 工具链和依赖库放置位置。
   - 如何只编译、如何传输、如何远程运行。
   - 常见问题：OpenCV/ncnn/pkg-config/开发板 SSH/内核路径。

2. CMake 建议改为目标化、可配置构建

   位置：

   - `main/CMakeLists.txt:14-15`
   - `main/CMakeLists.txt:23-30`
   - `main/CMakeLists.txt:52-54`
   - `main/CMakeLists.txt:59-80`

   问题：

   - `aux_source_directory` 会把 `example/src` 下所有 Demo 都编进主程序，不利于控制入口和依赖边界。
   - `-O3 -ffast-math` 全局启用。控制算法、IMU、PID、图像阈值中如果依赖 NaN/Inf、浮点顺序或精度，`-ffast-math` 可能让问题变隐蔽。
   - `if(NCNN_FOUND) add_definitions(-DLQ_HAVE_OPENCV)` 条件不匹配，应该基于 `OPENCV_FOUND` 定义 OpenCV，基于 `NCNN_FOUND` 定义 ncnn。
   - `link_directories` 是全局方式，建议改成 `target_link_directories` 或直接链接完整库路径。

   建议改法：

   - 明确列出主程序源文件、库源文件、用户源文件。
   - Demo 单独做成 `lq_demo_xxx` target，或者通过 CMake 选项只选一个 Demo。
   - 编译选项写到 target 上：

   ```cmake
   target_compile_options(${PROJECT_NAME} PRIVATE -O2 -Wall -Wextra)
   target_compile_definitions(${PROJECT_NAME} PRIVATE
       $<$<BOOL:${OPENCV_FOUND}>:LQ_HAVE_OPENCV>
       $<$<BOOL:${NCNN_FOUND}>:LQ_HAVE_NCNN>
   )
   ```

3. 工具链路径不应提交为个人绝对路径

   位置：

   - `main/toolchain_path.cmake:1`
   - `.vscode/settings.json:2`
   - `driver/Makefile:2`
   - `driver/TFT18_Driver/Makefile:2`

   问题：

   - 文件里曾经写死个人工作目录和内核目录。
   - 换一台机器或换一个目录，构建容易失败。

   建议改法：

   - `main/toolchain_path.cmake` 作为 `build.sh` 生成文件，不提交。
   - CMake 使用标准 `CMAKE_TOOLCHAIN_FILE`。
   - 驱动 Makefile 使用环境变量覆盖：

   ```make
   KDIR ?= $(LQ_KERNEL_DIR)
   CROSS_COMPILE ?= loongarch64-linux-gnu-
   ```

   并在 README 中说明：

   ```bash
   export LQ_KERNEL_DIR=/path/to/linux-4.19-loongson-embeded
   make -C driver KDIR="$LQ_KERNEL_DIR"
   ```

4. `clean_project.sh` 删除范围过大，建议加保护

   位置：

   - `main/clean_project.sh:19`
   - `main/clean_project.sh:33-37`

   问题：

   - 脚本会 `rm -rf` 构建目录，也会删除 `../tools/...` 下的工具链和依赖库。
   - 一次误运行可能导致重新下载/解压大量文件。

   建议改法：

   - 默认只清理 `main/build`。
   - 删除工具链和依赖库需要显式参数，例如 `./clean_project.sh --deps --toolchain --yes`。
   - 删除前校验真实路径必须在项目目录内，并打印将要删除的清单。

5. `build.sh` 不建议自动 `sudo apt install`

   位置：

   - `main/build.sh:91-120`
   - `main/build.sh:130-170`

   问题：

   - 构建脚本自动执行 `sudo apt update/install`，在 CI、离线环境或教学环境中都容易造成不可控副作用。

   建议改法：

   - 改成“检测缺失并提示安装命令”。
   - 提供单独的 `scripts/install_deps_ubuntu.sh`，让用户明确执行安装。

6. 驱动目录存在重复脚本和重复 I2C 公共代码

   位置：

   - `driver/*/scp_send.sh` 多个文件内容完全相同。
   - `driver/i2c_*_driver/inc/lq_i2c_read_write_drv.h` 多份完全相同。
   - `driver/i2c_*_driver/src/lq_i2c_read_write_drv.c` 多份基本相同。

   建议改法：

   - 抽出 `driver/common/`：

   ```text
   driver/common/inc/lq_i2c_read_write_drv.h
   driver/common/src/lq_i2c_read_write_drv.c
   driver/scripts/scp_send_ko.sh
   ```

   - 每个具体传感器驱动只保留设备差异部分。
   - 顶层 `driver/Makefile` 统一传入公共 include 路径。

7. `libraries/drv/src/lq_canfd.cpp` 避免用 `system()` 拼命令

   位置：

   - `libraries/drv/src/lq_canfd.cpp:68-73`
   - `libraries/drv/src/lq_canfd.cpp:95`

   问题：

   - `ifname` 被拼到 shell 命令里，虽然当前可能只传 `can0/can1`，但接口层没有限制输入。
   - `strcpy(ifr.ifr_name, ifname.c_str())` 没有限长保护。

   建议改法：

   - 至少校验接口名只允许 `[A-Za-z0-9_.:-]` 且长度小于 `IFNAMSIZ`。
   - 替换为 `snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str())`。
   - 更彻底的做法是使用 netlink/libsocketcan 配置 CAN，而不是调用 shell。

8. 资源管理类需要补齐拷贝/赋值语义

   位置：

   - `libraries/drv/src/lq_reg_gpio.cpp:85-127`
   - `libraries/drv/src/lq_reg_gpio.cpp:215-228`
   - `libraries/drv/src/lq_udp_client.cpp:35-66`
   - `libraries/drv/src/lq_udp_client.cpp:306-318`

   问题：

   - `ls_gpio::operator=` 直接覆盖当前映射和引用计数，没有先释放原资源。
   - `lq_udp_client::operator=` 也有类似问题。
   - `lq_udp_client` 默认构造后析构会把引用计数从 `0` 减到 `-1`。

   建议改法：

   - 使用 `std::shared_ptr` 搭配自定义 deleter 管理 `mmap` 和 socket。
   - 或者禁用拷贝，只允许移动：

   ```cpp
   ls_gpio(const ls_gpio&) = delete;
   ls_gpio& operator=(const ls_gpio&) = delete;
   ls_gpio(ls_gpio&&) noexcept = default;
   ls_gpio& operator=(ls_gpio&&) noexcept = default;
   ```

9. `lq_map_addr.cpp` 错误处理建议改掉 `assert("xxx")`

   位置：

   - `libraries/drv/src/lq_map_addr.cpp:15-27`

   问题：

   - `assert("open /dev/mem error")` 中字符串字面量永远为真，不能真正触发断言。
   - 当前虽然后面 `exit(EXIT_FAILURE)`，但错误信息不够准确，也不利于上层决定是否降级。

   建议改法：

   - 使用 `perror` 或项目自己的 `lq_log_error` 打印 `errno`。
   - 底层库尽量返回错误或抛异常，不直接 `exit` 整个进程。

10. UDP 图传返回值和协议可以更清晰

    位置：

    - `libraries/drv/src/lq_udp_client.cpp:256-283`

    问题：

    - `udp_send_image()` 分包发送 JPEG，但最后返回的是最开始长度包的发送字节数 `4`，不是整张图片发送总字节数。
    - UDP 分包没有帧 ID、包序号、总包数，接收端在丢包或乱序时很难恢复。

    建议改法：

    - 返回累计发送字节数。
    - 包头增加 `frame_id`、`packet_index`、`packet_count`、`payload_len`。
    - 接收端按帧重组，超时丢弃旧帧。

### P2：中长期优化，提升工程质量

1. 控制参数集中配置

   位置示例：

   - `user_app/src/my_motor.cpp:32-36`
   - `user_app/src/my_motor.cpp:131-145`
   - `user_app/src/my_motor.cpp:274-279`
   - `user_app/src/image_cv.cpp:24`
   - `user_app/src/image_cv.cpp:82-84`
   - `user_app/src/image.cpp:65-112`

   建议：

   - 把 PID、限幅、摄像头尺寸、HSV 阈值、基础速度、死区等集中到 `user_app/include/car_config.hpp`。
   - 区分“硬件参数”“赛道参数”“调试参数”。
   - 后续可支持从 JSON/INI/命令行加载，方便赛前调参。

2. 主循环和控制线程建议使用更稳定的定时策略

   位置：

   - `main/main.cpp:12-23`
   - `user_app/src/my_motor.cpp:200-229`
   - `user_app/src/my_motor.cpp:409-426`

   建议：

   - 用 `std::chrono::steady_clock` 和 `sleep_until` 减少周期漂移。
   - 记录控制周期最大值、平均值、超时次数。
   - 如果开发板允许，可将关键控制线程设置为实时调度，并谨慎控制线程内 I2C、printf、图像处理等耗时操作。

3. 信号处理不要直接做复杂停机动作

   位置：

   - `main/main.cpp:64-70`
   - `user_app/src/my_motor.cpp:459-464`
   - `libraries/drv/src/lq_signal_handle.cpp:15-24`

   建议：

   - 信号处理函数只设置原子退出标志。
   - 主循环检测退出标志后执行 `emergency_stop()`、释放摄像头、join 控制线程。
   - 避免在 signal handler 中调用复杂 C++ 对象方法、`printf`、`exit`。

4. 头文件保护宏避免使用双下划线

   位置示例：

   - `main/main.hpp:1`
   - `user_app/include/my_cascade_pid.hpp:1`
   - `user_app/include/motor.hpp:1`
   - `libraries/drv/inc/lq_map_addr.hpp:1`

   问题：

   - `__MAIN_HPP`、`__MOTOR_H__` 这类双下划线命名属于实现保留标识符，建议避免。

   建议改成：

   ```cpp
   #ifndef LQ_MAIN_HPP_
   #define LQ_MAIN_HPP_
   ```

5. 建议建立最小自动检查

   建议新增：

   - `scripts/check_format.sh`：运行 `clang-format --dry-run`。
   - `scripts/check_build.sh`：只配置/编译主程序。
   - `scripts/check_driver_build.sh`：驱动 `make W=1`。
   - `.github/workflows/` 或本地 CI 脚本，至少检查 CMake 配置和基础编译。

   对当前工程，哪怕没有真实硬件，也可以先做：

   - 语法编译。
   - 头文件自包含检查。
   - 核心纯算法单元测试，例如 PID、图像边界扫描、UDP 包头打包/解包。

## 建议的改动顺序

1. 先修 `main/build.sh` 的 `${BASH_SOURCE[0]}` 和 `BOARD_IP="${1:-}"`，确保“仅编译”真的只编译。
2. 修 `main/main.cpp` 的 Demo 入口和线程 `detach/join` 问题，恢复可控主流程。
3. 修 `motor.hpp` 与 `my_motor.cpp` 的编码器变量命名，避免跨文件状态失真。
4. 修 `MyMpu6050::init()` 的失败返回，避免 IMU 异常时闭环误启动。
5. 新增 `.gitignore`，把 `build*`、`.o/.ko/.cmd`、`tools/`、`toolchain_path.cmake` 从源码管理中剥离。
6. 重构 CMake：主程序、库、Demo 分 target；不要把所有 Demo 默认编入主程序。
7. 抽出 `driver/common` 和 `driver/scripts`，减少 I2C 驱动公共代码与发送脚本重复。
8. 逐步修资源管理类、CANFD 配置、UDP 图传协议和信号处理。

## 可直接落地的文件级清单

| 文件 | 建议修改点 |
| --- | --- |
| `main/build.sh` | 修正 `BASE_SOURCES` 为 `BASH_SOURCE`；`BOARD_IP` 改从 `$1` 读取；补 `set -euo pipefail`；自动安装依赖改为提示；远程运行脚本的重定向和 `kill -9` 改温和停机。 |
| `main/CMakeLists.txt` | 去掉全局 `aux_source_directory`；拆分主程序/库/Demo target；修正 `LQ_HAVE_OPENCV` 定义条件；减少全局 `-ffast-math`。 |
| `main/main.cpp` | 去掉临时 Demo 早退；建立运行模式选择；修复 `detach` 后 `join`；信号处理只置位退出标志。 |
| `main/toolchain_path.cmake` | 作为生成文件加入 `.gitignore`，不要提交个人绝对路径。 |
| `.vscode/settings.json` | `cmake.sourceDirectory` 改成 `${workspaceFolder}/main`，避免个人路径。 |
| `user_app/include/motor.hpp` | 与实现统一编码器变量命名；减少不必要 `extern`。 |
| `user_app/src/my_motor.cpp` | 参数集中到配置头；控制周期统计；减少控制线程内阻塞操作；补充 IMU 异常降级日志。 |
| `user_app/src/my_mpu6050.cpp` | `id != 0x68` 默认返回 `false`；`new/delete` 改 `std::unique_ptr` 或值成员。 |
| `user_app/src/image_cv.cpp` | 摄像头设备、曝光、HSV 阈值、ROI 参数配置化；避免固定 `/dev/video0`。 |
| `libraries/drv/src/lq_canfd.cpp` | 避免 `system()` 拼 shell 命令；限制 `ifname` 长度；用 `snprintf` 替代 `strcpy`。 |
| `libraries/drv/src/lq_map_addr.cpp` | `assert("xxx")` 改成真实错误处理；底层库不要直接 `exit`。 |
| `libraries/drv/src/lq_reg_gpio.cpp` | 重新设计拷贝/赋值资源语义，或禁用拷贝只允许移动。 |
| `libraries/drv/src/lq_udp_client.cpp` | 修默认构造析构引用计数；赋值前释放旧 socket；`udp_send_image()` 返回累计字节数并增加包序号。 |
| `driver/Makefile` 和各子驱动 `Makefile` | 统一 `KDIR`、`ARCH`、`CROSS_COMPILE`；公共代码抽到 `driver/common`；构建产物只进入 `build/ko` 且不提交。 |
| `driver/*/scp_send.sh` | 合并为一个公共脚本，子目录不再复制同一份。 |

## 总结

这个项目的核心能力已经比较丰富，真正需要优先处理的是工程组织和入口控制：让“构建、选择 Demo、运行主控、传输开发板”这几件事变得稳定、可预测。建议先从 `build.sh`、`main.cpp`、`.gitignore` 和 CMake 入手，这几处改完后，后续调参和新增功能会轻松很多。
