# Intake and shooter bring-up

## Current configuration

- Intake pivot: CAN 21, 60:1 reduction, 118 degree stowed hard-stop reference, 0 degree deployed target, 80 A smart current setting.
- Pickup rollers: CAN 2/37, 80 A smart current setting per motor.
- Intake feeder: CAN 28/35, 40 A smart current setting per motor.
- Shooter flywheel: CAN 39 leader with 34/26/29 followers, 80 A smart current setting per motor, 80 rps near target and 95 rps far target.
- Shooter indexer: CAN 36/22, 40 A smart current setting per motor.
- Hood: CAN 3 brushed controller, 20 A current setting.

These current settings are controller configuration values, not guaranteed instantaneous current bounds or continuous thermal ratings. The velocity gains remain initial bring-up values.

## Controls and configuration

Left trigger deploys the intake and runs the pickup, feeder and indexer while held. Release stops the rollers and returns the pivot target to stowed. Right trigger requests shooting speed, then starts the indexer when measured flywheel speed and hood readiness qualify; release returns both to idle. Test mode supports the normal Xbox bindings unless an explicit diagnostic is selected.

SPARK velocity commands use ordinary velocity control when acceleration is unspecified, and MAXMotion velocity only with a positive acceleration profile. Pivot position uses ordinary position control. Encoder units, firmware feedforward, gain readbacks and configuration errors are logged. Shooter speed readiness compares measured speed against the current shooter state, avoiding the prior idle-readiness race.

The SPARK adapter uses duty-cycle units for P/I/D and voltage units for feedforward. Shooter P is 0.08 / 12 duty per radian/second, corresponding to the intended initial 0.08 V per radian/second at nominal 12 V. Flywheel kV is 0.11 / (2*pi) V per radian/second; indexer kV is 0.12 / (2*pi).

The shooter enables optional per-motor telemetry under AngularControllers/<CAN ID>: encoder RPM/rotations, current, bus voltage, applied duty/voltage, temperature, faults, warnings and connection status. Poll timestamps are not CAN-frame freshness guarantees. Supply current reported by AngularIOSparkFlex is an estimate from motor output current times applied duty.

## Encoder reference on restart

A normal startup seeds the pivot encoder to the configured 118 degree hard-stop reference. For a warm restart that must retain an already established encoder coordinate, create /tmp/bordie-angular-keep-reference-21 on the roboRIO, owned by lvuser:ni, before deployment. The one-shot marker is consumed by startup and skips both initialization resets. It preserves the existing controller coordinate; it does not establish the physical reference. Explicit angle resets remain available.

## Physical status and remaining work

The intake deployed in live testing. The shooter command reached its controller, but recorded attempts did not achieve shooting speed. The latest session recorded severe voltage sag, controller fault recovery, and motor-current peaks above the configured smart-current setting. The operator reported that the drum turns by hand and feeder/shooter motors move in opposing directions despite needing coordinated motion.

Motor grouping and follower polarity still require physical verification. The code retains the existing follower relationships; no unverified inversion change is included. Hood readiness uses a travel-time fallback when its position sensor is absent, so readiness alone does not verify physical hood travel.

The earlier live shooter build used P=0.08 in the adapter's duty units. The corrected P scaling and optional per-motor telemetry in this commit have not been deployed or physically verified. No speed ramp is implemented.

## Verification and recordings

Java regression tests cover Test and Teleop Xbox triggers, release behavior, readiness sequencing, angular units, configuration transitions, simulator behavior and diagnostic gating. Python tests cover the diagnostic tooling without enabling the robot.

Read-only NetworkTables recording is provided by tools/capture_intake_diagnostics.py. Live recordings and deployment/build logs are retained in the operator workspace and excluded from this source commit. Physical mechanism behavior remains a separate validation step from unit tests.
