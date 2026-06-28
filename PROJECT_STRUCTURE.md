# Project Structure

## Top-Level Layout

```text
.
├── main/                  # Car application entry and build script
├── user_app/              # Control, image, IMU, encoder and NCNN code
├── example/               # Vendor demos and staged tuning tools
├── libraries/             # Vendor application/common/driver libraries
├── driver/                # Kernel drivers and deployment helpers
├── tools/                 # Cross toolchain and runtime dependencies
├── docs/                  # Datasheets and notes
├── gpio_proj/             # GPIO hardware watchdog daemon
├── autoboot/              # Board-side gpio_manager installation package
└── PROJECT_STRUCTURE.md
```

## Safety Architecture

The car application and `gpio_manager` communicate through `/gpio_proj_shm`:

```text
car control thread
  └─ gpio_protect_heartbeat_pulse() every 10 ms
       writes heartbeat_counter

gpio_manager
  └─ checks the counter every 1 ms
       if it is unchanged for more than 50 ms:
         keeps PWM channels enabled with compare = ARR + 1 (safe stop for the active-low driver)
         sets GPIO21..24 LOW
         sets GTIM CCR2 to 0
         sets GPIO61 LOW
```

The protected pins match the current two-motor wiring:

| Function | Car application pin | Watchdog protection |
| --- | --- | --- |
| Left motor PWM | `ATIM_PWM0_PIN81` | ATIM CCR1 / GPIO81 |
| Right motor PWM | `ATIM_PWM1_PIN82` | ATIM CCR2 / GPIO82 |
| Left direction | `PIN_21` | GPIO21 LOW |
| Right direction | `PIN_22` | GPIO22 LOW |

The car application also performs a local `emergency_stop()`. It clears speed targets and PID output, keeps PWM enabled, and writes duty 0. This is intentional for the active-low motor driver: disabling PWM could leave its input floating.

## Current Car Mode Behavior

`main/main.cpp` supports `--mode=car`, `tune`, `encoder-test`, `encoder512`, `vision-debug`, `speed-tune`, and `speed-return`.

For `--mode=car`:

1. `gpio_manager` readiness is required by default. Use of `--unsafe-dev-mode` bypasses that check only for bench diagnostics and must not be used on track.
2. MPU6050 initialization and bias calibration must succeed before motor control starts.
3. Camera initialization must succeed before motor control starts.
4. The control thread stops the vehicle on stale vision data, watchdog loss, invalid encoder feedback, reverse encoder feedback, or sustained high PWM without wheel motion.
5. The current race entry deliberately sets `g_enable_element_process = false`; enable and validate element processing before using a task that requires elements.

## Board Deployment Order

1. Build and install `gpio_manager`.
2. Start and verify `gpio-manager.service`.
3. Build/deploy the car application.
4. Run the car application in a safe mode first, then `--mode=car` only after the safety checks pass.

```bash
# GPIO watchdog daemon
cd gpio_proj
./build.sh manager

# On board: install/restart service
cd /root/autoboot
./install.sh
systemctl status gpio-manager --no-pager

# Car application
cd /path/to/repo/main
./build.sh
```

## Autostart Package

The current `autoboot/install.sh` installs and starts only `gpio-manager.service`. It explicitly disables `screen.service`; screen startup is not part of the present safety deployment path.

## Main Build Commands

```bash
cd main
./build.sh                         # Build only
./build.sh --clean                 # Clean build
./build.sh <board-ip>              # Build and copy
APP_ARGS="--mode=tune" ./build.sh <board-ip> -r
./build.sh <board-ip> -r           # Build, copy and run car mode
```
