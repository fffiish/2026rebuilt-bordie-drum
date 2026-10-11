# Shooter: corrected inertia and delay characterization

The current full-voltage WPILib bang-bang controller reaches speed and recovers from a synthetic disturbance, but does **not** reliably meet the chosen ±10% steady-speed criterion across feedback-delay assumptions. This is conditional simulation evidence, not a hardware validation.

## Requirement and candidate

Evaluate the existing controller using the user's corrected **total rotating inertia of 6.5 lb·in² = 0.0019021577472898 kg·m²**, about the drum axis. Keep the controller and hardware configuration unchanged. The previous simulation inertia was 0.012 kg·m²; the older first-order test was not an inertia-based physical model.

Baseline revision: `faef7fd210325d5331fe1a5a0251eccd8d957a71`, branch `bang-bang-drum`. Candidate changes the simulation inertia and adds a characterization harness. Source hashes and test totals are captured in [the results manifest](shooter-inertia.results.json).

Acceptance criteria were established before running: first production `atSpeed` assertion within 2 seconds, sampled true speed within ±10% of target between 2–7 seconds, first recovery crossing of 90% target within 1 second after a synthetic speed loss, and zero motor request on stop. Startup uses the existing 80% readiness threshold; it does not mean settled at target. Recovery is a first crossing, not sustained settling.

## Model and unresolved hardware facts

The test runs the production shooter and WPILib scheduler in enabled teleop at 20 ms, with an independently integrated 1 ms plant. Four WPILib `DCMotor.getNeoVortex` motors drive the drum with the code's assumed 1:1 gearing. The model uses ideal 37 A per-motor phase-current limiting and open-circuit coast at zero command. Torque accelerates the stated inertia; viscous drag is calibrated to the code comment's 242 rad/s at 4.46 V holding point. That comment is not a newly measured log. Drag is swept at 0.5, 1, and 1.5 times nominal.

Motor specifications are documented by [REV](https://docs.revrobotics.com/brushless/neo/vortex). The project's [pinned REVLib 2026.0.5 source archive](https://maven.revrobotics.com/com/revrobotics/frc/REVLib-java/2026.0.5/REVLib-java-2026.0.5-sources.jar) documents quadrature defaults of 100 ms measurement period and depth 64. The current source sets UVW period/depth but does not explicitly configure quadrature period/depth. The effective Vortex/Flex filter settings and firmware must therefore be read back on hardware before treating any case as its actual latency.

The velocity estimator uses quantized position at an assumed 7168 counts/revolution, a finite-difference measurement window, then a moving average updated every 1 ms. That averaging cadence and encoder resolution are assumptions, not a reproduction of proprietary firmware. **100 ms/depth 64 is a conservative sensitivity case, not a measured fixed encoder delay.** Faster 10 ms/depth 2 and 5 ms/depth 1 cases are hypothetical explicit configurations, not settings applied to the robot.

CAN velocity delivery is modeled every 20 ms with 0/10 ms phase. Added command delay is 0/20 ms. Fixed bus voltages of 12/8 V bracket voltage conditions; battery sag, REV current-limit transients, mechanical compliance, unequal motors, and actual ball contact are not modeled. At 7 seconds, true speed is set to 80% of the lesser of current speed and target. No source WPILOG was used or modified; all feedback in this harness is simulation generated. Results cover normal and soft targets; normal/far share a target. Eject and complete match behavior are outside this sweep.

## Observed results

144 cases ran: 3 filters × 2 targets × 2 voltages × 2 CAN phases × 2 command delays × 3 drag levels.

| Assumed estimator | Startup pass | Steady ±10% pass | Recovery pass | Stop pass |
| --- | ---: | ---: | ---: | ---: |
| 100 ms / depth 64 | 48/48 | 0/48 | 48/48 | 48/48 |
| 10 ms / depth 2 | 48/48 | 24/48 | 48/48 | 48/48 |
| 5 ms / depth 1 | 48/48 | 33/48 | 48/48 | 48/48 |

Nominal examples below use 12 V, nominal drag, CAN phase 0, and zero extra command delay. These are sampled true drum speeds, not the delayed reported encoder values.

| Target | 100 ms / depth 64 range | 10 ms / depth 2 range |
| --- | ---: | ---: |
| Normal: 3321.5 RPM | 2969.2–4013.5 RPM | 3227.4–3478.1 RPM |
| Soft: 2958.2 RPM | 2637.0–3765.8 RPM | 2884.8–3116.5 RPM |

Low inertia permits fast acceleration, so full power can continue past the target while reported velocity catches up. Faster estimation improves the nominal result, but some added-delay cases still exceed the band. The result supports proceeding with measured encoder timing and bounded controller tuning; it does not support claiming robust ±10% regulation for the current setup.

The JUnit harness deliberately records acceptance failures as data. Passing JUnit means model invariants and execution succeeded, not that the table's speed criteria all passed. A separate physics check verifies initial torque/inertia acceleration, zero-current coast decay, and the calibrated holding point. The full existing test suite also passed; consult the manifest for counts.

## Reproduce and review

Use the project's WPILib Java 17 JDK (the installed newer JDK is incompatible with the project's Lombok). With that JDK selected, run from the robot repository:

```sh
bash gradlew test --tests 'frc.robot.subsystems.shooter.ShooterInertiaSimulationTest' --console=plain
python3 validation/analyze_shooter_inertia.py
```

The reusable [scenario](shooter-inertia.scenario.json), [summary CSV](shooter-inertia-summary.csv), analyzer, and manifest are retained for review. Raw traces and the static comparison plot are generated under ignored `logs/generated/shooter-inertia/`. No deployment, live CAN changes, or hardware actuation occurred. The controller was not tuned.
