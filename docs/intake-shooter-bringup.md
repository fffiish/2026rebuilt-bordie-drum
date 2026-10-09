# Intake and shooter bring-up

## Current configuration

- Intake pivot: CAN 21, 60:1 reduction, 118 degree stowed hard-stop reference, 0 degree deployed target, 60 A normal smart current setting, 80 A during deployment, 120 A secondary setting.
- Pickup rollers: CAN 2/37, 80 A smart current setting per motor.
- Intake feeder: CAN 28/35, 40 A smart current setting per motor.
- Shooter flywheel: CAN 39 leader, CAN 34 same-direction follower, CAN 26/29 opposed followers, 40 A smart and 60 A secondary current settings per motor, 80 rps near target and 95 rps far target. Encoder measurement period is 10 ms with averaging depth 2.
- Shooter indexer: CAN 36/22, 40 A smart current setting per motor.
- Hood: CAN 3 brushed controller, 20 A current setting.

These current settings are controller configuration values, not guaranteed instantaneous current bounds or continuous thermal ratings. The velocity gains remain initial bring-up values.

## Controls and configuration

Autonomous or teleop entry calls `deployOnce()`. Deployment makes the arm's resting target 0 degrees and requests 80 A smart current. Reaching the existing 5 degree arrival tolerance, or the existing two-second motion timeout, advances the sequence to request 60 A. The merge preserves the source branch's behavior: setting the resting target marks deployment as done for `deployOnce()`, and interruption does not add a 60 A cleanup request. A current request is asynchronous and does not wait for configuration readback.

Left trigger runs the pickup and intake-feeder rollers while held, without moving the arm or running the shooter indexer. Release stops those rollers. Right trigger starts shooter spin-up and arm agitation concurrently: three 0-to-20-degree cycles with 0.35 seconds per stroke, then a 90 degree raised target. The shooter indexer starts after the existing measured-speed and hood-readiness gate; it does not wait for deployment or agitation. Releasing the trigger returns shooter/indexer to idle and the arm to its resting target. Intake rollers and shooting can run together. Right bumper reverses intake rollers and unjams the indexer.

Test mode retains normal Xbox bindings unless a diagnostic is selected, and does not deploy automatically. Use the `Intake/DeployTest` dashboard command for explicit deployment in enabled Test mode with no diagnostic selected. `DeployIntake` and `StowIntake` remain available as named auto commands; stowing changes the resting target back to 118 degrees. Existing diagnostic selection retains control of outputs.

Start resets field heading while preserving field translation: 0 degrees for blue or unknown alliance, 180 degrees for red. It works while disabled. Other drive bindings remain unchanged.

SPARK velocity commands use ordinary velocity control when acceleration is unspecified, and MAXMotion velocity only with a positive acceleration profile. Pivot position uses ordinary position control. Encoder units, firmware feedforward, gain readbacks and configuration errors are logged. Shooter speed readiness compares measured speed against the current shooter state, avoiding the prior idle-readiness race.

The SPARK adapter uses duty-cycle units for P/I/D and voltage units for feedforward. Shooter P is 0.01 / 12 duty per radian/second, corresponding to 0.01 V per radian/second at nominal 12 V. Flywheel kV is 0.11 / (2*pi) V per radian/second; indexer kV is 0.12 / (2*pi). The existing disabled-only gain/reference configuration and recovery paths are retained; the current-only setter changes smart current without resetting or persisting settings.

The shooter enables optional per-motor telemetry under AngularControllers/<CAN ID>: encoder RPM/rotations, current, bus voltage, applied duty/voltage, temperature, faults, warnings and connection status. Poll timestamps are not CAN-frame freshness guarantees. Supply current reported by AngularIOSparkFlex is an estimate from motor output current times applied duty.

## Encoder reference on restart

A normal startup seeds the pivot encoder to the configured 118 degree hard-stop reference. For a warm restart that must retain an already established encoder coordinate, create /tmp/bordie-angular-keep-reference-21 on the roboRIO, owned by lvuser:ni, before deployment. The one-shot marker is consumed by startup and skips both initialization resets. It preserves the existing controller coordinate; it does not establish the physical reference. Explicit angle resets remain available.

## Physical status and remaining work

Earlier encoder-branch recordings show intake deployment and shooter attempts with voltage sag, controller fault recovery and low measured speed. The later `feeder-shooter` commit `b639769` reports that the shooter motors work and spin up at the 40 A setting, while the feeder does not work and flywheel feedback still reads zero RPS. These are source-branch observations; this merge does not resolve the feeder or speed-feedback issue.

The merged code preserves the follower relationships from `feeder-shooter`. Hood readiness retains its travel-time fallback when the position sensor is absent. No robot deployment or physical operation is part of this merge.

No speed ramp, additional interlock, current-confirmation gate, timeout, lockout, retry restriction or deployment-before-shooting requirement is added by this integration.

## Verification and recordings

Java regression tests cover Test and Teleop Xbox triggers, release behavior, readiness sequencing, angular units, configuration transitions, simulator behavior and diagnostic gating. Python tests cover the diagnostic tooling without enabling the robot.

Read-only NetworkTables recording is provided by tools/capture_intake_diagnostics.py. Tracked recordings from `diagnostics-2026-10-09` are preserved in `diagnostics/`; existing untracked workspace recordings are not added. Physical mechanism behavior remains a separate validation step from unit tests.
