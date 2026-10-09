# Drive speed and steering diagnostics — team 1899

## Starting evidence

The previous enabled log showed roughly 1% typical wheel speed spread across 13
samples with equal commands and aligned steering. That does not establish a hardware
fault. Steering outputs reversed repeatedly, and targets sometimes changed with the
operator's request. The normal loop averaged about 229 ms. Fixed-target tests are
required to separate target changes, feedback sign, overshoot, and sensor timing.

## Changes

- Dashboard gains now apply only while disabled and are read back from each controller.
  Drive P starts at 0.005 and kV at 0.112 volts per wheel radian/second. Steering starts
  at P 0.1, I 0, D 0. REV 2026 uses `closedLoop.feedForward.kV` in volts per configured
  velocity unit: [REV FeedForwardConfig](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/feedforwardconfig).
- Runtime firmware requests are removed from the drivetrain loop and limited to once
  per second for other REV mechanisms, with bounded request timeout and no retries.
  Vision publishes timing for each read/decode stage.
- Steering seeds only while disabled. Failed initialization/configuration and unhealthy
  telemetry inhibit module outputs. Applied inversion, conversions, and wrapping are checked.
- Test mode bypasses the normal scheduler and bindings and directly stops intake pivot,
  intake rollers, feeder, indexer, flywheel, and hood. Only the drivetrain diagnostic can
  command movement. All diagnostic runs require a real operator holding Xbox A on slot 0.
- Open-loop steering is capped at 0.25 V; steering PID is capped at ±10% duty. Drive
  voltage is capped at 1.5 V and drive PID at ±15% duty. Timed runs cannot repeat while
  the activation button stays held. Release, disable, failed health, stale samples,
  excessive travel, wrong encoder direction, or oscillation stops the run.

Calibration offsets and motor inversion constants in `TunerConstants.java` are preserved.
No CANcoder configuration changes are made.

## Operator procedure

Keep the robot secured with wheels raised and an operator ready to disable it.
On SmartDashboard, set the following while **disabled**. Test mode may be selected
after setup; movement still requires Driver Station Test to be enabled:

| Key under `DriveDiagnostics/` | Meaning |
|---|---|
| `Prepare` | Set true while disabled to apply/read back diagnostic output limits |
| `Stage` | `FEEDBACK`, `STEER_POSITIVE`, `STEER_NEGATIVE`, `DRIVE_OPEN`, `DRIVE_CLOSED`, or `NONE` |
| `Module` | 0 front left, 1 front right, 2 back left, 3 back right, 4 all |
| `Magnitude` | Open drive: 0.5 / 1.0 / 1.5 volts; closed drive: 0.25 / 0.5 m/s |

Wait for healthy sensors and qualified Test-loop timing: at least 100 samples, median
below 25 ms, 95th percentile below 40 ms. Enable Test, release A once, then hold A for
one run. Release A after each run. Changing selections during a run cancels it.

1. `FEEDBACK`: each module individually; at most 150 ms or 2° travel. Absolute and
   relative changes must agree. Motion too small to establish direction fails.
2. `STEER_POSITIVE` and `STEER_NEGATIVE`: each module individually after feedback passes.
   Step 5°, settle within 1 s, hold within ±1° for 5 s, then return and repeat the hold.
   Growing error or repeated significant reversals abort. Tune only from these traces.
3. `DRIVE_OPEN`: continuous voltage hold on one or all four modules, steering unpowered; hard stop at 20 s.
   Steering remains unpowered. Compare speed, acceleration, current, and direction.
4. `DRIVE_CLOSED`: all four only after both steering checks pass on every module;
   ramp to target over 1 s and stop within 3 s. Hold the captured headings. Require
   last-second mean speed spread ≤5% and each wheel within 10% of the target.

Gain changes invalidate previous steering checks. Disable before changing gains.
Finish disabled; set Stage `NONE` and Prepare false while disabled to restore normal
output ranges. A robot-program restart also clears diagnostic pass history.

## Capture and analysis

`python tools/capture_drive_diagnostics.py build/drive-run.json --seconds 30`
is read-only. It subscribes to timestamped diagnostic traces, sensor status, gains,
outputs, current, and Driver Station state. Install `websocket-client` and `msgpack`
if using another computer. Robot WPILOG recordings retain the same traces.

REVLib 2026 does not expose actual relative-encoder status timestamps. Their logged
ages remain NaN; status health uses immediate REV error checks under a 100 ms frame
timeout. CANcoder age comes from its real signal timestamp. Reconstructed motor RPM
comes from the configured wheel conversion and gear ratio, not an independent sensor.
Test timing and normal robot timing must be reported separately because Test excludes
the other subsystem polling paths.

## Validation status

The remote bounded drive test was run over Ethernet on team 1899 with four drive motors
selected and steering unpowered. All configuration and sensor-health flags stayed healthy.
The 0.5 V, two-second pulse reported wheel-speed means of 0.2081, 0.2032, 0.2081, and
0.2101 m/s (3.34% spread, within the 5% criterion). At 1.0 V, the first pulse reported
0.1040, 0.1005, 0.0999, and 0.0471 m/s (64.7% spread). A repeat confirmed the mismatch:
0.1026, 0.0991, 0.0922, and 0.0492 m/s (62.2% spread).

During the repeat, module 3 (back right, drive CAN ID 1) reported about 0.45 V applied,
50.7 motor RPM, and 13.4 A, while the other modules reported 0.76–0.96 V, 125–131 RPM,
and 17–21 A. This is a repeatable drive-output/speed discrepancy, not a basis for changing
PID gains or encoder conversion yet. No gain, steering-zero, or polarity changes were made
from these tests. After each run the diagnostic became inactive and all four drive and
steering outputs read zero; the Driver Station itself still reported enabled Test mode.

The initial PID defaults are in place and read back from the controllers, but final gains,
physical polarity, speed equality over the operating range, and elimination of steering
oscillation remain unverified.
