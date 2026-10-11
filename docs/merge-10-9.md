# merge-10-9 integration

## Included branches

Repository: `fffiish/2026rebuilt-bordie-drum`.

| Branch | Source commit |
| --- | --- |
| encoder | 9cfd35bc60423152458dcfe9410f75265a56eb46 |
| feeder-shooter | b6397690b014a68294f141b123580256347d9384 |
| controls | 535d5d1ff84b12110084175bed5c8dbf3167315f |
| oriented | cb02ed850b25071fbaca087a9e22ece01efb166d |
| diagnostics-2026-10-09 | 7f9f551b866270617c5f83ed6db9532922b3c373 |

The integration starts from feeder-shooter, merges diagnostics, then oriented. Encoder is already an ancestor of feeder-shooter; controls is already an ancestor of oriented. Main is not separately merged or updated.

## Selected behavior

- Independent intake pivot and roller commands, automatic autonomous/teleop deployment, simultaneous pickup and shooting, and concurrent shooting agitation from controls/oriented.
- Intake pickup leaves the shooter indexer uncommanded, retaining feeder-shooter behavior. The existing speed/hood readiness gate still precedes indexer feeding.
- Encoder calibration, diagnostic policies, watchdogs, reference handling and direct output stopping are retained. Shooter wiring, follower directions, gains and filtering come from feeder-shooter.
- Pivot smart current is 80 A during deployment and 60 A at the end of the source deployment sequence, including its existing two-second motion timeout. Secondary current remains 120 A. Cancelling the sequence does not add a 60 A cleanup request.
- Test mode deployment is explicit through `Intake/DeployTest`. Both `DeployIntake` and `StowIntake` named commands remain available.
- Start-button heading reset preserves translation and uses the existing alliance behavior.
- The diagnostic summary script reads split roller-state topics with fallback to the old combined-state topic, preserving compatibility with existing recordings.

No new safety behavior was requested or added: there are no additional interlocks, current-confirmation gates, timeouts, lockouts, retry restrictions or deployment-before-shooting requirements. Detailed controls and configuration are in [intake and shooter bring-up](intake-shooter-bringup.md).

## Verification

- WPILib 2026 JDK, local worktree outside OneDrive.
- `gradlew.bat test jar -x spotlessApply --console=plain`: passed, 84 Java tests, zero failures/errors.
- `python -B -m unittest discover -s tools -p 'test_intake_*.py'`: passed, 15 Python tests, including three diagnostic-summary compatibility cases.
- `gradlew.bat build -x spotlessApply -x spotlessJsonCheck -x createVersionFile --console=plain`: passed, including all 84 Java tests and Java/Gradle/documentation formatting checks. The already-generated version file was reused after formatting.
- The initial full `spotlessCheck` reported inherited recording JSON formatting violations, including changes from numeric `Infinity` to a string. Those recordings remain unchanged; the JSON formatting check is excluded from the successful build above. This integration does not alter the repository's formatting configuration.
- Independent controls, hardware and integration reviews found no remaining functional merge issues or unrequested safety additions.

No robot deployment or physical operation was performed for this integration. The source feeder-shooter commit's reported feeder issue and zero-RPS feedback remain separate physical observations, not problems resolved by these software checks.
