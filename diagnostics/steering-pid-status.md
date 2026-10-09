# Steering PID investigation

Scope: steering stability and fixed-target response. Drive-speed work is complete and excluded.

## Verified state

- Source checkout: `C:\Users\otgra\2026rebuilt-bordie-drum`, branch `encoder`, clean at `6eb3588` when inspected. The FRC-Bordie workspace contains diagnostic captures and has no committed source checkout.
- Live robot: `10.18.99.2`, build metadata SHA `2891647709ff2dbd6fc27794a191d30b731fdc27`, with uncommitted changes reported by the deployed build. The live build must not be equated to current source HEAD.
- All four steering controllers read back P approximately 0.1 and D 0. Source configures and verifies I 0, but a separate applied-I topic is not published.
- Fresh five-second preflight: loop median 20.016 ms, nearest-rank p95 21.239 ms, maximum 29.031 ms over 248 published periods. This was outside Test mode, not Test timing qualification.
- All four modules reported seeded steering, connected relative/absolute sensors, and healthy controller configuration. Diagnostic output limits were false.
- Preflight Driver Station state was attached, enabled outside Test, with no e-stop. All drive voltages were zero; steering voltages were approximately 0.10–0.13 V.

Evidence: `pid-preflight.json` records capture UTC, live topic metadata, values, and histories.

## What this does and does not establish

The saved three-minute Teleop observation had steering-voltage reversals on all four modules, but its subscription omitted steering target and error topics. It cannot distinguish target changes from overshoot at a fixed target. Its loop median was 20.016 ms and p95 21.351 ms, with a 102.303 ms maximum.

The live preflight showed nearly constant requested angles and approximately 4.8–6.0 degree steering errors. With the configured radian position units and P 0.1, a five-degree error produces about 0.87% duty, or 0.105 V at 12 V. Insufficient torque to overcome friction is a plausible explanation for stationary residual error. This is an inference, not a measured friction threshold or a selected final gain.

Source inspection confirms dashboard gains are applied only while disabled, checked by controller readback, and steering reseeding is inhibited while enabled. Existing diagnostics bypass normal target optimization, keep drive output zero for steering tests, and cap closed-loop steering to 10% duty after preparation.

## Authorized physical sequence

The operator confirmed secure support, clear wheels and area, and readiness at Driver Station, and authorized bounded steering checks. Preparation was verified while disabled: all four modules acknowledged diagnostic limits, configuration health, P 0.1, and D 0. Test timing qualified at approximately 20.0 ms median and 21.1 ms p95.

1. While disabled, prepare diagnostics and remote activation; verify all output-limit and configuration acknowledgements, gains, sensor health, and freshness.
2. Enable Test at Driver Station and require 100 fresh Test-loop periods with median below 25 ms and p95 below 40 ms.
3. One module at a time: feedback pulse at no more than 0.25 V, ending by 150 ms or 2 degrees. Require absolute and relative motion agreement before closed-loop steps.
4. Test +5 degrees and -5 degrees, returning to the captured starting angle. Each phase must settle within one degree in one second and stay there for five seconds, under 10% steering duty. Stop on growing error, repeated swings, stale signals, or timing/watchdog loss.
5. Choose further P adjustments only from the measured response. Add D only if fixed-target traces demonstrate overshoot requiring damping. Keep I zero for initial diagnosis.
6. Verify all drive and steering outputs return to zero. Finish disabled and restore preparation state.

## Powered checks completed

| Module | Check | Recorded result |
| --- | --- | --- |
| Front left | Feedback pulse, at most 0.25 V | PASS: relative delta 2.365 degrees; absolute delta 2.285 degrees; difference approximately 0.080 degrees. The sampled stop occurs at the travel limit; the final recorded travel exceeds 2 degrees by 0.365 degrees. |
| Front left | +5 degree step, P 0.1, I 0, D 0 | STOP at approximately one second: final error 4.965 degrees. Relative movement approximately 0.035 degrees; maximum steering voltage 0.106 V. Target held fixed during the commanded phase. |

Evidence: `pid-20261008-230522-m0-feedback.json` and `pid-20261008-230605-m0-steer_positive.json`.

All four drive voltages remained zero in both recorded tests, and all eight drive/steering voltage readings returned to zero afterward. No negative step, return/hold qualification, or other module test has been performed yet.

The control tool initially saw the feedback result nonce and reason before its updated pass flag, producing a contradictory console result. The independent completed recording shows LastRunPassed true and a PASS status for that exact nonce. The local stage wrapper now verifies final recording and stopped outputs without repeating movement; it verifies the result belongs to a new nonce, selected module, and stage.

P 0.3 was subsequently applied and verified on all four controllers while disabled. I remained zero and D remained zero. This was a diagnostic trial value, not a completed PID tune.

The operator reported not seeing the initial short motor pulses and explicitly requested movement that could be seen. Four existing bounded feedback pulses were then run in succession on each module separately, accumulating 11.07, 12.13, 12.22, and 12.22 degrees of absolute steering travel on modules 0 through 3. Relative encoder changes agreed within approximately 0.31 degrees. Peak steering voltage stayed below 0.25 V; every drive voltage stayed zero; all eight voltages returned to zero after each sequence. Each four-pulse sequence spanned roughly one second. These are telemetry results, not visual confirmation. `steering-jog-summary.json` and the four `pid-visible-*.json` captures retain the evidence.

## Latest operator instruction: use supplied values and skip movement tests

The operator supplied a photo specifying turning/azimuth P 1.0, I 0, D 0, with continuous angle input from -pi to pi, and explicitly ended further movement tests.

The source steering default in `ModuleIOSpark.java` is updated from P 0.1 to P 1.0. I is already zero, D defaults to zero, and controller position wrapping is already enabled with limits -pi and pi. Drive gains are outside this change's scope.

`gradlew.bat test jar -x spotlessApply --console=plain` succeeded with all 24 existing software tests passing. The source change is limited to the steering P default and its explanatory comment. Deployment and final readback are waiting for the robot to be disabled. All eight live voltage readings were zero in the last pre-deployment observation.

These are operator-selected gains. Physical PID stability is not validated; the operator explicitly requested skipping further movement tests.
