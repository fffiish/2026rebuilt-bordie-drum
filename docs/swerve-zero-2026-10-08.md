# Swerve straight-position calibration — team 1899

The operator placed all four wheels at the desired straight-ahead positions. Those positions
are the reference for this calibration. No powered drive or steering check was requested.

## Saved offsets

Captured over Ethernet at **2026-10-09 03:13:39.339530 UTC** (2026-10-08 20:13:39 PDT).
Offsets are rotations, measured from the CANcoder signal as currently configured on the device.
The software subtracts each offset; CANcoder magnet offsets and sensor directions were not changed.

| Wheel | Module | Drive ID | Steer ID | CANcoder ID | Software offset (rotations) | Capture variation (degrees) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Front left | 0 | 32 | 6 | 20 | -0.496740004624 | 0.17578125 |
| Front right | 1 | 33 | 5 | 17 | 0.000488281250 | 0 |
| Back left | 2 | 7 | 52 | 10 | 0.001436121331 | 0.087890625 |
| Back right | 3 | 1 | 25 | 4 | 0.000976562500 | 0 |

The offsets are circular means of 17 fresh snapshots per module over approximately 4.8 seconds,
excluding the initial retained NetworkTables snapshot. Circular averaging handles wraparound.
Every accepted snapshot had connected drive/steer controllers, a connected CANcoder, a valid
CAN timestamp, a successfully seeded steering encoder, and disabled robot state. Maximum CAN
sample age was 0.012740 seconds; all eight drivetrain applied voltages were zero.

The front-right drive inversion override remains false. All other motor inversion settings,
gear ratios, module assignments, and CANcoder device settings are preserved.

## Read-only diagnostic stream

`ModuleIOSpark` publishes a timestamped snapshot while disabled at
`/AdvantageKit/RealOutputs/Drive/Calibration/CANcoder<ID>`. The timestamp ensures that a steady
angle is distinguishable from stale telemetry. This logger makes no motor or encoder writes.

Each double array contains these fields, in order:

1. FPGA time, seconds
2. Raw CANcoder absolute angle, rotations (includes existing device configuration)
3. CAN sample age, seconds
4. CAN timestamp valid (1/0)
5. CANcoder connected (1/0)
6. Drive controller connected (1/0)
7. Steering controller connected (1/0)
8. Offset-adjusted absolute angle, radians
9. Relative steering encoder angle, radians
10. Drive applied voltage
11. Steering applied voltage
12. Robot disabled (1)
13. Steering encoder seeded (1/0)

For future capture, discard the retained initial snapshot and require at least two seconds of
fresh, increasing FPGA timestamps, CAN ages at most 0.1 seconds, all health flags true, zero
motor outputs, and less than one degree of circular angle variation. Capture all four modules
together while physically held at the desired reference. Do not infer zero from a disconnected
encoder or rewrite CANcoder device offsets in addition to these software offsets.

## Verification

`gradlew.bat compileJava -x spotlessApply` and `gradlew.bat deploy -x spotlessApply` succeeded.
Formatting was excluded to preserve unrelated files. Compilation reported existing deprecation
warnings. No automated test suite or powered motor test was run.

The first verification was captured at **2026-10-09 03:15:56.659615 UTC**, after deployment.
The second was captured at **2026-10-09 03:16:53.866298 UTC**, after another program restart
using the standard roboRIO `frcKillRobot.sh -t -r` script. The robot program PID changed from
2703 to 2914, with the deployed JAR checksum unchanged:
`926f868935cae5e237c716be0ff227ad019afcce1adde79dc5e34764609cbc41` (SHA-256).

| Wheel | After deploy: max absolute error | After deploy: max relative error | After restart: max absolute error | After restart: max relative error |
| --- | ---: | ---: | ---: | ---: |
| Front left | 0.056871 degrees | 0.031021 degrees | 0.056871 degrees | 0.031021 degrees |
| Front right | 0 degrees | 0 degrees | 0 degrees | 0 degrees |
| Back left | 0.077551 degrees | 0.010341 degrees | 0.077551 degrees | 0.077551 degrees |
| Back right | 0.087891 degrees | 0 degrees | 0 degrees | 0 degrees |

Each successful verification covered 20 fresh snapshots per module over approximately five
seconds. Every snapshot passed the same connection, timestamp, disabled-state, and zero-voltage
checks as the initial capture. All absolute and relative errors were within the one-degree
acceptance tolerance. An initial capture during the second startup had less than two seconds of
samples and was rejected; verification was repeated after startup completed.

The deployed build date reported by telemetry was `2026-10-08 23:15:13 EDT`. Its Git metadata
references parent commit `2772495388301aa161092cdaadf7122eb1c75786`; these calibration changes
were built from the working tree. The diagnostic deployment cleared the displayed Emergency
Stopped state; Driver Station and robot telemetry subsequently confirmed Teleoperated Disabled.
No enable command or powered motor command was issued for calibration. Existing loop-overrun
and non-drivetrain Spark Max ID 3 timeout messages remain outside this calibration.

Accepted snapshots and summaries are stored in [swerve-zero-2026-10-08.json](swerve-zero-2026-10-08.json).
These checks establish the encoder reference; they do not validate powered steering or driving.

## Front-left drive polarity correction

After calibration, the operator reported that the front-left drive wheel appeared reversed.
An independent sub-agent found that setting its former approximately -179-degree steering
reading to zero changes the swerve optimizer's speed sign: a forward command previously
optimized to a reversed drive speed at the half-turn heading; it now uses positive speed at
the calibrated zero. The initial zero-angle checks did not catch this rolling-direction change.

The correction adds `kFrontLeftDriveMotorInverted = true` and uses it only for FrontLeft
(drive CAN ID 32). The captured steering offset remains -0.496740004624 rotations. The
front-right drive override remains false, back-left remains false, and back-right remains true.
Steering inversion, CANcoder settings, and all four steering offsets remain unchanged.
The earlier statement that every drive inversion was preserved describes the initial capture;
this front-left override supersedes that setting.

Live telemetry before the correction showed aligned steering readings of approximately
16.05, 16.61, 16.27, and 16.79 degrees, with disabled/emergency-stopped state and zero
drivetrain applied voltage. These were the positions after the operator's intervening check;
they were not used to recapture the straight-ahead reference. The sub-agent edited only the
front-left drive configuration; the parent agent handles compilation and deployment.

Compilation and deployment succeeded with `-x spotlessApply`. Post-deployment telemetry was
captured at **2026-10-09 03:23:51.566999 UTC**: 20 fresh snapshots per module over approximately
4.9 seconds, connected controllers/encoders, CAN sample ages below 0.012 seconds, disabled state,
and zero drive/steer applied voltage. The steering readings remained near their pre-deployment
16-degree positions; the stored zero reference was not changed. The program restart again cleared
the displayed Emergency Stopped state, and Driver Station confirmed Teleoperated Disabled.

Local and deployed JAR SHA-256 hashes matched:
`71a876489070b648a9aefa8942cdccc8b1db08a84a9683a4b1093a873ae1bd30`.
The deployed build date was `2026-10-08 23:23:05 EDT`. Evidence is in
[front-left-drive-fix-2026-10-08.json](front-left-drive-fix-2026-10-08.json).
No powered check was performed; the corrected rolling direction and feedback sign remain
unverified physically. No robot enable or powered motor command was issued by either agent.
