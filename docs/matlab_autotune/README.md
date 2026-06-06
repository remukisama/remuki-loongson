# MATLAB Autotune Bridge

This directory contains MATLAB R2016a-compatible scripts for automated Loongson
SmartCar tuning.

Smoke test in MATLAB:

```matlab
cd('\\wsl$\Ubuntu-22.04\home\zoumaguanbei\SmartCar_Project\Loongson_2k300_301_Library-master\Loongson_2K300_301_LIB\docs\matlab_autotune')
run_autotune_smoke
```

Safe smoke test from PowerShell, with timeout protection. A local copy has also
been written to `C:\tmp\run_lq_matlab_smoke_safe.ps1`; prefer that local script.
Do not run the `.ps1` directly from `\\wsl$`; MATLAB R2016a or PowerShell can
hang before creating the log.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File C:\tmp\run_lq_matlab_smoke_safe.ps1
```

Build-only test from MATLAB:

```matlab
lq_autotune('mode','build','stage','T11')
```

Board run from MATLAB:

```matlab
lq_autotune('mode','board','stage','T11','boardIp','192.168.5.100','iterations',3)
```

Board mode copies `lq_autotune_stage.txt` to the board before launching the
program, then waits for the requested `[Txx_SUMMARY]` line in `main.log`.

The C++ entrypoint checks `lq_autotune_stage.txt` or
`/tmp/lq_autotune_stage.txt`. When the file contains `T11`, `T06`, `T07`,
`T10`, or `FLOW_LOW_SPEED`, the board runs that tuning stage first and prints
machine-readable `[Txx_SUMMARY]` lines. Without the file, the normal `main.cpp`
flow is unchanged.
