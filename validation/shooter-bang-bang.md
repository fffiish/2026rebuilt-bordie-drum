# Shooter WPILib bang-bang validation

The flywheel now uses WPILib BangBangController on the roboRIO. The four SPARK Flex
motors receive a positive 12 V request below target and zero at or above target;
the hardware IO clamps the request to available bus voltage. REV velocity PID is
unused. All four motors are configured for coast with no output ramp, retaining
37 A smart and 60 A secondary current limits and the existing follower polarity.

Normal/far targets are 6.4/0.0184 rad/s (about 3322 RPM), soft is 5.7/0.0184
(about 2958 RPM), eject is 1.25/0.0184 (about 649 RPM), and idle is zero. These
are provisional values from the old voltage presets, not measured shot calibration.
Feeding starts at 80% of the chosen target for every connected motor. Once started,
feeders continue through speed dips, as before. Configuration faults, pending
configuration, invalid/disconnected feedback and samples older than 100 ms inhibit
flywheel output and readiness. Healthy feedback permits recovery. Disabled or
interrupted commands and immediate stop request zero output; immediate stop also
clears the latched shooter state so the default command cannot restart it.

## Evidence

Base revision: `38884337fd3b89721a69c196451f6b533dd36c49` on `merge-10-9`.
Candidate is the uncommitted working tree; source SHA-256 hashes and test totals
are recorded in `shooter-bang-bang.results.json`.

Run with the project's installed WPILib Java 17 runtime:

```sh
bash gradlew test --tests 'frc.robot.subsystems.shooter.*' --console=plain
bash gradlew test --console=plain
```

The focused suite passed 19 tests. The final full suite passed all 109 tests,
including angular IO, intake, indexer-related configuration, drive and shooter
regressions. Tests cover on/off boundaries, all presets, lack of REV velocity
requests, 80% readiness, mirrored follower signs, fault/stale feedback and recovery,
A-button preset changes, release/interruption, autonomous spin-up, disabled and
immediate stops. Two preexisting tests were updated to the intentionally changed
speed cap and zero REV gains after the first full run exposed their old expectations.
An initial run with system Java 21 could not compile the project's old Lombok
processor; using installed WPILib Java 17 resolved it without dependency changes.

The deterministic plant test uses 20 ms scheduler steps, a powered time constant
of 0.5 s and coast-drag time constant of 2 s. Powered equilibrium speed follows
volts/0.0184 rad/s; zero voltage produces coast with no electrical braking.
All four plant scenarios pass startup <=2 s, steady speed within +/-10% of target
for 5 s, recovery to >=90% within 1 s after a 20% speed loss, and zero commanded
output within one cycle after stop.

| Target | Supply | Time to 80% | Steady RPM range | Recovery to 90% |
|---|---:|---:|---:|---:|
| Normal | 12 V | 0.30 s | 3288.4–3435.4 | 0.08 s |
| Soft | 12 V | 0.26 s | 2928.9–3083.8 | 0.06 s |
| Normal | 8 V | 0.54 s | 3304.8–3339.9 | 0.16 s |
| Soft | 8 V | 0.46 s | 2929.8–3001.4 | 0.12 s |

`shooter-bang-bang.scenario.json` describes the repeatable plant scenario. Gradle
JUnit XML and HTML reports are generated under `build/test-results/test` and
`build/reports/tests/test`; these generated reports are not committed.
No source WPILOGs were read or modified. Replay was not used because changed
motor output would change subsequent speed inputs.

## Limits and hardware handoff

The plant results establish software behavior within a declared model. The test
plant does not model REV current limiting, actual flywheel inertia, encoder lag,
battery current or ball contact. The shared production AngularIOSim still models
electrical braking at zero voltage despite coast configuration, so it is not the
coast-validation oracle. Human review and controlled hardware tests are required
before deployment: verify all motors coast, leader/follower polarity, speed
calibration, current draw, battery sag, overshoot and speed recovery during feeding.
The selected 12 V cap and removed output ramp increase startup/recovery effort.
Nothing was deployed, enabled or actuated on robot hardware.
