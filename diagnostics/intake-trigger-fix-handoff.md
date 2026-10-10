# Intake left-trigger fix

Confirmed stowed hard stop: 118 degrees. Pivot reduction: 60:1.

Source checkout: C:\Users\otgra\source\bordie-intake-calibration
Changes also applied to main C:\Users\otgra\2026rebuilt-bordie-drum without replacing concurrent web configuration integration.

Corrected build deployed to team 1899, 10.18.99.2. Robot command points at /home/lvuser/bordie-intake-calibration.jar. Deployed SHA256: 74669695ab99583cbe5c674d88d9f0966f080a4801419c60a3073c26183b13c3.

Normal Xbox left trigger uses intakeFuel: deploy pivot + pickup rollers + feeder rollers + indexer. Release stops rollers and commands stow.

Test mode previously skipped CommandScheduler; diagnostics also unconditionally stopped the intake in Test. Test now disables LiveWindow and explicitly enables CommandScheduler, and unselected diagnostics do not overwrite enabled normal control. Rollers with zero profile acceleration use plain velocity control. Real nonzero gains restored from the previously running intake branch artifact and seeded in acknowledged hardware startup configuration: pivot P=2, V=.5, G=.35; pickup/feeder V=.12.

Verification: isolated 69 Java tests pass; main 84 Java tests pass with existing concurrent configuration-inventory gate excluded (full task fails that stale inventory check). Deployed live readback confirms118-degree position/forward limit, all intake configReady=true, pivot P/V/G readback correct, output cap1, no diagnostic selection, scheduler active.

Physical Test LT trial: rollers turned; pivot did not deploy. trigger-fix-30degree-test.json confirms LT1, kIntaking, goal0, closed-loop, all configs ready; native MAXMotion reference0 and applied0V throughout. Binding is functioning; native pivot profile/controller path failed.

Subsequent fix: AngularIOSparkFlex.setAngle uses direct kPosition instead of MAXMotionPosition; no profiled trajectory is requested. Position command REV result logged to RealOutputs/AngularControllers/21/PositionCommandResult. ReferencePos reports requested goal in this mode. Build/test69 pass; updated artifact deployed (hash above). Waiting on subsequent operator test>=30deg, with live recorder direct-position-live-test.json. No physical movement success claimed yet.


Direct-position live trial completed: native PositionCommandResult=kOk; LT activates bothrollergroups. Pivotfirstholdcurrentmedian40.073A, voltage median-.7278V, busminimum8.1538V; only.4135degreepivottravel. Acrossallthreeholds maximumawaytravel1.909degrees, so30degdeploymentNOTachieved. Configurationreadytrue/errorsempty. SourceSmartcurrent40A/secondary80A; globalcurrentpeak80.403A. Controllerappliescurrent-limitedoutputbutphysicaldeploymentstillfails. RawNTJSON direct-position-live-test.json; aligned20msCSV pivot-voltage-current-test.csv. Pendingoperatorinspectionofpivotmotor/gearbox/latch/coupling.


Latest operator direction: remove the 30-degree test criterion. The required behavior is full configured travel: hold Xbox left trigger to deploy from the118-degree stowed reference to0degrees while bothrollergroupsrun; release stopsrollersandcommands118-degree stow. There is no30-degree travelcap in the deployed normal control path. Future verification should measure full endpoint travel, not stop at30degrees. The recorded current-limited failure remains unresolved; do not claim full physical deployment.


Urgent loaded-hopper follow-up: operator changed battery; baseline12.45V. PivotSmartCurrent40->60A and realrollerKV corrected .12/(2pi) V/(rad/s) in bothdevice/subsystemconfigs. Operator reports unloadedpivotdeploysbutpushinghopperstalls. 60A recording firsttwoattemptsreach4.1/4.8deg; loadedattemptsstall116degwith60A.

Nextapprovedbuild pivotSmartCurrent80A, secondary120A; userbrieflyaskedstopdeploymentafteritcompletedthenexplicitlyauthorizedcurrent80Abuild. RemoteartifactSHA256 f6fd23ceb830763295d5775704c3b3fa7b2abd18fb60e27be4c4aff8d1e9730c. Sourcechangesalsosyncedmaincheckout.69Javatestspass. Currentrecording intake-hopper-80amp-test.json: firstpress118->6.887deg, currentmax81.062A, minbus8.315V; secondpress117.386->9.526deg, max78.132A/minbus8.352V. Confighealthinterruptiononsecondattemptclearedafterdisable. Waitingoperatorphysicalconfirmationthatloadedhopperextends; do notclaimcompletephysicaldeploybeforethatconfirmation. Fulltarget0deg,nothirtydegcriterion.


Pickupstallfollow-up: operator reportsballsjammingpickuprollers andrequests60A. ChangedonlypickupCAN2/37 SmartCurrentLimit40->60A; mastersecondary80Aretained. FeederCAN28/35 remains40A, pivotCAN21 remains80A/secondary120A.69Javatestsandjarpass; deployedhash635db0df5212a26b1e30fabd6e7190d7e107e8bb2b5b4c18f6e4421c3cea15c7. Operatorconfirmedarmupclosehardstopandexplicitdeployment. LivepickupConfigReadytrue/ConfigError empty;DSenabledandrecordingpickup-60amp-ball-test.json. Physicalball-clearanceconfirmationpending. Existingrightbumperouttakecommand reversespickup/feeder/indexerforjamclearance.


Pickup80Afollow-up explicitlyrequestedconditionalonreasonableNEOVortexsetting; REVFAQlists80Adefaultmostapplications. ChangedpickupCAN2/37 smart60->80A; mastersecondary80->120A. Feeder40A,pivot80Aunchanged. Userexplicitlyrequestedstartingcurrentdeployedarmpositionratherthanreturnhardstop. Addedone-shotwarmrestartreferencepreservationinSparkIO: consume/tmp/bordie-angular-keep-reference-{masterId}, skipconstructorencoderseedandfirstAngularSubsystemnoargreset; normalcoldboot/no-markerstillconfiguredreference. Createdonly21markerownedlvuserbeforedeploy; liveStartupReferencePreserved21true,actualangle4.6437degandpickupConfigReadytrue/errorsempty.69Javatestspass. RemotejarSHA7ae02009a3d8a448921a46da9a3d3ebbd5bddc304e6931c643ea8f4bbba4e67e. Sourcechangesbothcheckouts. Recordingpickup-80amp-ball-test.json nowactive; jamclearancephysicalresultstillpending.
