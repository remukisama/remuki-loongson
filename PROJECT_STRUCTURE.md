# Project Structure

This repository keeps the original Longqiu smart-car application layout, and
adds the verified `loongson-car-system` safety layers around it.

## Top-Level Layout

```text
.
├── main/                  # Current car application entry, build script, CMake
├── user_app/              # Current car control, image, motor, IMU, NCNN code
├── example/               # Longqiu demos and staged tuning tests
├── libraries/             # Longqiu application/common/driver libraries
├── driver/                # Kernel drivers and driver transfer scripts
├── tools/                 # Toolchain and third-party runtime libraries
├── docs/                  # Datasheets and project notes
├── gpio_proj/             # Verified GPIO watchdog daemon project
├── autoboot/              # Verified board-side autostart package
└── PROJECT_STRUCTURE.md   # This structure guide
```

## Role Mapping To `loongson-car-system`

| This repository | `loongson-car-system` role | Notes |
| --- | --- | --- |
| `main/`, `user_app/`, `example/`, `libraries/` | `car_project/` | Existing monolithic car application. It now sends the same watchdog heartbeat protocol as `car_project`. |
| `gpio_proj/` | `gpio_proj/` | Copied verified GPIO watchdog daemon source. It creates `/gpio_proj_shm` and writes safe hardware values on heartbeat timeout. |
| `autoboot/` | `autoboot/` | Copied verified board-side install package. It installs `gpio_manager` and `screen_app` systemd services. |
| Not migrated | `screen_project/` | The prebuilt `screen_app` is included in `autoboot/bin`; full screen source is not needed for motor safety. |
| Not migrated | `pc_tuner/`, `Matlab/` | Tuning tooling is separate from the immediate motor watchdog fix. |

## Safety Architecture

The current car application and the watchdog communicate through the verified
shared memory protocol:

```text
current main/user_app
  └─ gpio_protect_heartbeat_pulse()
       writes /gpio_proj_shm heartbeat_counter every 10ms

gpio_proj/gpio_manager
  └─ checks heartbeat_counter every 1ms
       if unchanged for >50ms:
         ATIM CCR1..CCR4 = 0
         GTIM CCR2 = 0
         GPIO21..24 = LOW
         GPIO61 = LOW
```

The protected pins match the current two-motor wiring:

| Function | Current app pin | Watchdog protected pin |
| --- | --- | --- |
| Left motor PWM | `ATIM_PWM0_PIN81` | ATIM CCR1 / GPIO81 |
| Right motor PWM | `ATIM_PWM1_PIN82` | ATIM CCR2 / GPIO82 |
| Left direction | `PIN_21` | GPIO21 LOW |
| Right direction | `PIN_22` | GPIO22 LOW |

The current app also keeps a local fallback in `emergency_stop()`: it clears
speed targets and PID output, keeps PWM enabled, and writes duty 0. This avoids
letting a self-made driver board see floating PWM inputs after a PWM-disable.

## Board Deployment Order

Use this order on the Loongson board:

1. Install and start `autoboot/` so `gpio_manager` is running first.
2. Verify `systemctl status gpio-manager` is active.
3. Start or deploy the current car application from `main/build.sh`.

Expected service order:

```text
gpio_manager  ->  screen_app  ->  current car application
```

If `gpio_manager` is not running, the current car application still builds and
runs, but only local emergency-stop protection remains. External 50ms hardware
takeover is unavailable until `/gpio_proj_shm` exists.

## Current Application Entry Warning

At the time this guide was added, `main/main.cpp` still contains an early NCNN
demo entry:

```cpp
lq_ncnn_image_infer_demo();
return 0;
```

For real driving tests, confirm whether this early return should be disabled so
the program reaches `motor_Init()`, `timer_thread_1()`, and the main control
loop. The watchdog heartbeat is initialized before that early return, but the
car control loop itself will not run while the demo return is active.

## Build Commands

Current car app:

```bash
cd main
./build.sh
```

GPIO watchdog daemon:

```bash
cd gpio_proj
./build.sh manager
```

Board-side autostart package:

```bash
scp -r autoboot root@<board-ip>:/root/autoboot
ssh root@<board-ip>
cd /root/autoboot
chmod +x install.sh
./install.sh
```
