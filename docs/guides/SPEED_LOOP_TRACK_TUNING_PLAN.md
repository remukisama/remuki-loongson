# Speed Loop Track Tuning Plan

## Current safe state

- Board binary hash: `f8ad08e2d9befda0d0af520fda91aed6642dbae8c26071c70e4b4a6b3f0e22c0`.
- `--mode=speed-return` is disabled and only sends a disabled UDP event after stopping both motors.
- `--mode=speed-tune` is forward-only. It does not enter a return/reverse phase.
- Automatic reverse PWM settings are zero until reverse encoder feedback is proven trustworthy.

## Next real-car test limits

Use only `./main --mode=speed-tune` after the car is manually placed at the start.

- Sine peak speed: 0.55 m/s.
- Forward active window: 1200 ms.
- Forward distance limit: 1.2 m.
- PWM cap: 4200.
- Launch pulse: 3200 for 120 ms.
- Overspeed stop: 1.20 m/s.
- Wheel mismatch stop: 0.25 m/s.
- Max target speed remains capped by the global 2.0 m/s guard.

## Required UDP evidence per round

Collect `speed_tune` UDP logs from port 8081 and inspect:

- `valid` and `badMs`: encoder feedback must stay valid during forward motion.
- `rawL/rawR` and `encL/encR`: raw speeds must not show impossible spikes.
- `target` vs `encL/encR`: judge overshoot, lag, settling, and symmetry.
- `pwmL/pwmR`: avoid long saturation near the PWM cap.
- `fwd`: confirm distance stays well inside the 6 m lane.
- `reason`: acceptable first-round stop reasons are `forward_time_done`, `auto_return_disabled`, or `settle_timeout` after motors are already stopped.

## Tuning rule

Do not increase speed or duration until one short round proves stable encoder feedback and controlled motor response. Tune PI only from valid forward logs. Reverse/return must remain disabled until a separate encoder diagnosis proves reverse speed is trustworthy.
## Log analysis helper

After each short forward-only run, analyze the collected UDP log with:

```bash
python3 analyze_speed_tune_log.py /mnt/c/tmp/<speed_tune_log>.log
```

The helper reports forward encoder validity, target/speed peak, RMSE, wheel mismatch, PWM saturation, stop reason, and conservative next `kSpeedPidP` / `kSpeedPidI` suggestions. Do not apply suggestions when `suggest_reason=encoder_invalid_do_not_tune_pi`; fix encoder validity first.