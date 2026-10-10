# Controller latency diagnosis

Captured October 8, 2026, approximately 8:49 PM Pacific, using read-only Ethernet/SSH/NT4 inspection of team 1899's physical robot at `10.18.99.2`.

## Finding

The robot program has severe main-loop stalls. Blocking CAN device-presence checks are the leading cause. Two configured controllers report disconnected: intake roller follower CAN 37 and hood SPARK MAX CAN 3. The deployed code queries firmware for each mechanism controller every loop, including disconnected devices. Timeout/retry waits are strongly implicated by the long, consistent subsystem durations and the offline statuses. Individual calls were not instrumented during this diagnosis, so attributing each entire subsystem delay specifically to firmware calls remains an inference.

## Measurements

| Measurement | Result | Evidence |
| --- | --- | --- |
| Actual robot loop interval | Median 366.084 ms; mean 368.245 ms; p95 387.305 ms; range 351.412–391.479 ms | 55 consecutive differences of robot-generated `/AdvantageKit/Timestamp` values in a 20-second NT4 capture |
| Intended loop period | 20 ms | Robot constants and live WPILib overrun messages |
| Mechanism input reads, aggregated | Median 177.746 ms; mean 178.107 ms | 56 live `Timing/AngularSubsystems/InputUpdateMS` samples |
| Hood periodic method | Median 148.892 ms; mean 149.221 ms | 136 watchdog epoch records in the captured console tail |
| Drivetrain input reads | Median 14.302 ms; mean 14.995 ms | 56 live `Timing/Drive/InputUpdateMS` samples |
| Entire robotPeriodic method | Median 343.120 ms; mean 342.970 ms | 135 console watchdog epoch records |
| Ethernet ICMP round trip | 12/12 replies; maximum 1 ms; 0% packet loss | Local ping to roboRIO |
| CAN utilization | Approximately 56–62%; error counters zero in capture | Live `/AdvantageKit/SystemStats/CANBus` telemetry |

The 366 ms interval means approximately 2.7 robot updates per second instead of the intended 50. This is a major input-to-command bottleneck and is consistent with the reported 0.5–1 second perceived response. It does not independently measure the entire physical controller-to-wheel delay. Sampling was performed while disabled; enabled timing and controller transport delay were not measured. Ethernet ping is not the Driver Station control-packet trip-time measurement.

## Code and deployed version

The live robot reports Git SHA `2772495388301aa161092cdaadf7122eb1c75786`, build date `2026-10-08 23:23:05 EDT`, and disabled state. All eight drivetrain applied-voltage readings remained zero during capture.

The active development checkout is `C:\Users\otgra\2026rebuilt-bordie-drum`, branch `encoder`, HEAD `e8072262dae115930026404d3acf9da2e596bcf4`, with ongoing uncommitted changes from the separate **Clone Bordie Drum repository** chat. The checkout under `C:\Users\otgra\Documents\2026rebuilt-bordie-drum` is a different, clean `main` checkout and should not be used to infer current deployed behavior.

Code inspected from the robot-reported commit, not just the changing working tree:

- `lib/subsystem/angular/AngularIOSparkFlex.java`: `updateInputs` calls `setConnected` for each master and follower; `setConnected` calls `spark.getFirmwareVersion()` on every pass.
- `subsystems/shooter/HoodIOSparkMax.java`: `updateInputs` calls `actuators.getFirmwareVersion()` every pass; the configured hood ID is 3 and live `Hood/ControllerConnected` is false.
- `subsystems/drive/ModuleIOSpark.java`: eight more firmware queries per loop, two per module. Connected drivetrain reads are much smaller than the mechanism stalls.
- `commands/DriveCommands.java`: normal translation input is cubed after a 0.05 deadband. This can feel weak around the center but does not introduce a half-second timed delay. The normal joystickDrive path has no slew-rate limiter or explicit wait.

## Fix priorities

1. Replace per-loop firmware requests with cached health plus status-frame freshness/error checks. If firmware probing remains necessary, limit its frequency and bound timeout/retries. Measure per-device calls to confirm their contribution. REV documents CAN timeout behavior in [SparkBase.setCANTimeout](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkbase#setCANTimeout(int)).
2. Check power, CAN wiring, and configured IDs for intake follower 37 and hood 3. If those devices are intentionally absent, gate their hardware IO initialization. Hardware existence flags currently enable all mechanisms.
3. Confirm the separate chat's pending I/O changes cover both offline devices, all followers, and the drivetrain. Working-tree fixes were present during inspection but the captured robot was still running the earlier build. No improved latency has been verified.
4. Repeat timing after the fix, targeting median below 25 ms and p95 below 40 ms. Then perform one controlled end-to-end input response observation; do not infer physical response solely from network ping.

The console also repeatedly reports NavX disconnect/reconnect events: 36 disconnect messages in the saved tail. Investigate this separately because unstable gyro feedback can affect field-relative steering. Its connection to the controller delay was not established.

## Evidence files

- `live_latency.json`: raw timestamped NT4 data and topic definitions.
- `live_latency_summary.json`: per-topic statistics.
- `device_health.json`: robot-reported device connectivity, including intake follower 37 and hood controller.
- `robot_console_tail.txt`: captured roboRIO console tail.
- `console_timing_summary.json`: watchdog epoch statistics.
- `capture_latency.py`: read-only subscriber used for the 20-second capture.

No source edits, deployment, configuration writes, robot enabling, or powered tests were performed by this diagnosis.
