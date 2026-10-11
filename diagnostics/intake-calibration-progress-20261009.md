# Intake calibration progress — 2026-10-09

The operator confirmed a 60:1 pivot reduction and a stowed hard stop at 118 degrees. The deployed source now uses 118 degrees for the stowed position, startup reference, and forward soft limit. The floor/deployed angle of 0 degrees remains unmeasured.

Xbox left trigger now requests pickup and feeder rollers without changing the pivot command or intake target state. Release stops both rollers. Command tests verify pivot ownership is excluded and autonomous intake still deploys.

## Source and deployment

Original checkout: `C:\Users\otgra\2026rebuilt-bordie-drum`, branch `encoder`, base commit `2e57301c73c8c588091ea82488f46dacd8bf9e28`.

The original checkout is also being edited by another configuration-web-app task. Further calibration builds use isolated checkout `C:\Users\otgra\source\bordie-intake-calibration`, based on the last successfully built source embedded in the prior jar, plus the exact WPILib 118-degree conversion correction.

Running robot artifact: `/home/lvuser/bordie-intake-calibration.jar`.
Local and remote SHA-256 match: `c1fced8078f44ea616730c1a41a100848937b916fdd38ad446d807ac52f4aae3`.

59 Java tests passed for the deployed build. 12 Python control tests now pass; six new tests cover queued telemetry after the recorder joins. That Python-only fix does not require redeploying Java.

## First physical feedback trial

Recorded evidence: `intake-feedback-20261009-01.json`; compact metrics: `intake-feedback-20261009-01-summary.json`.

- Request nonce 1: -0.25 V, maximum 150 ms, 3 percent output cap, 10 A diagnostic cutoff.
- Live Test mode and loop timing were verified before activation.
- Actual active duration approximately 80 ms; stopped for reported pivot current exceeding 10 A.
- Peak reported motor output current: 10.2564 A.
- Applied voltage remained approximately -0.25 V.
- Encoder position changed from 117.999998 to 117.869418 degrees (delta -0.130581 degrees).
- Independent capture verified zero voltage after the abort. A separate stop command verified a fresh zero-voltage sample and retained diagnostic isolation.
- Driver Station subsequently reported disabled Test mode, zero applied voltage, and zero velocity.
- Physical arm direction/scale has not yet been confirmed by the operator. Tiny encoder motion could include backlash or deflection.

Independent Astra review found no Java current/voltage scaling error. The current magnitude remains unexplained by a simple motor model; supply current in these logs is a calculated estimate, not independent hardware measurement.

## Prepared next trial, not activated

While fresh disabled state was verified, settings were changed to -0.20 V for 100 ms, retaining the 10 A cutoff and 3 percent cap. Calibration/reference was retained without reseeding the encoder; the arm is no longer assumed to be physically touching the hard stop. RemoteActivation remains false. StopRequested was explicitly cleared while disabled for this prepared request.

Await the operator observation of the first pulse before further activation. The operator must enable Test mode for a follow-up. A repeat current abort or negligible physical motion requires checking the motor/load/controller current reporting. Do not raise the current cutoff merely to obtain a passing run.

Position PID tuning has not started. All real pivot and roller gains remain zero/unverified. Pickup rollers and feeder tuning follow pivot characterization. No final PID gains have been selected, committed, or pushed.
