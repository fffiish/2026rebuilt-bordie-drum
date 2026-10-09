package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayDeque;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

/** Bounded Test-mode diagnostics with physical or explicitly prepared remote activation. */
public final class DriveDiagnostics {
  private static final String KEY = "DriveDiagnostics/";
  private static final double STEP_RAD = Math.toRadians(5);
  private static final double HOLD_TOLERANCE_RAD = Math.toRadians(1);
  private static final double TRAVEL_LIMIT_RAD = Math.toRadians(2);
  private final Module[] modules;
  private final DriveDiagnosticPolicy.TimingWindow timing =
      new DriveDiagnosticPolicy.TimingWindow();
  private final DriveDiagnosticPolicy.DeadmanGate gate =
      new DriveDiagnosticPolicy.DeadmanGate();
  private final RemoteDiagnosticPolicy.RequestGate remoteRequests =
      new RemoteDiagnosticPolicy.RequestGate();
  private final DriveDiagnosticPolicy.OscillationGuard steeringGuard =
      new DriveDiagnosticPolicy.OscillationGuard();
  private final boolean[][] steeringPassed = new boolean[4][2];
  private final boolean[] feedbackPassed = new boolean[4];
  private final double[][] verifiedGains = new double[4][4];
  private final boolean[][] verifiedState = new boolean[4][4];
  private boolean gainsObserved;
  private final Object outputLock = new Object();
  private final Notifier watchdog;
  private volatile double hardDeadlineSec;
  private volatile double heartbeatSec;
  private volatile boolean watchdogRunning;
  private volatile boolean watchdogTripped;
  private volatile String watchdogReason = "";
  private volatile boolean remoteAllowed;
  private volatile boolean remoteActivation;
  private volatile double remoteHeartbeatReceiptSec = Double.NaN;
  private double observedRemoteHeartbeat;
  private boolean wasTestEnabled;
  private boolean wasTestMode;
  private final double[] startingAngles = new double[4];
  private final double[] startingAbsoluteAngles = new double[4];
  private final double[] targets = new double[4];
  private final ArrayDeque<double[]> steadySpeeds = new ArrayDeque<>();
  private Stage stage = Stage.NONE;
  private int selectedModule;
  private double magnitude;
  private double startSec;
  private double activeRunNonce;
  private double lastNowSec = Double.NaN;
  private double requestedSpeed;
  private boolean active;
  private boolean returnPhase;
  private boolean limitsRequested;
  private String status = "Idle: prepare while Test mode is disabled, then release Xbox A";

  private enum Stage {
    NONE,
    FEEDBACK,
    STEER_POSITIVE,
    STEER_NEGATIVE,
    DRIVE_OPEN,
    DRIVE_CLOSED
  }

  public DriveDiagnostics(Drive drive) {
    modules = drive.getDiagnosticModules();
    SmartDashboard.putBoolean(KEY + "Prepare", false);
    SmartDashboard.putString(KEY + "Stage", "NONE");
    SmartDashboard.putBoolean(KEY + "RemoteAllowed", false);
    SmartDashboard.putBoolean(KEY + "RemoteActivation", false);
    SmartDashboard.putNumber(KEY + "RemoteHeartbeat", 0);
    SmartDashboard.putNumber(KEY + "StartNonce", 0);
    Logger.recordOutput(KEY + "LastRunNonce", 0.0);
    Logger.recordOutput(KEY + "LastRunTimestampSec", 0.0);
    Logger.recordOutput(KEY + "LastRunPassed", false);
    Logger.recordOutput(KEY + "LastRunReason", "No completed run in this program instance");
    SmartDashboard.setDefaultNumber(KEY + "Module", 0);
    SmartDashboard.setDefaultNumber(KEY + "Magnitude", 0.5);
    Logger.recordOutput(KEY + "TraceColumns",
        "time,stage,module,requestedSpeed; per module: relativeAngle,absoluteAngle,target,error,"
            + "speed,driveVolts,turnVolts,driveAmps,turnAmps,driveRpm,turnRpm,snapshotTime,"
            + "absoluteAge,driveStatus,turnStatus,seeded,configurationHealthy,limitsApplied");
    watchdog = new Notifier(this::watchdogCheck);
    watchdog.setName("DrivetrainDiagnosticSafety");
    watchdog.startPeriodic(0.005);
  }

  /** Called once per loop after module input updates, in every Driver Station mode. */
  public void periodic(double nowSec, double loopPeriodMs) {
    boolean testMode = DriverStation.isTest();
    if (testMode != wasTestMode) {
      timing.clear();
      wasTestMode = testMode;
    }
    if (testMode) {
      timing.add(loopPeriodMs, nowSec);
    }
    boolean prepare = SmartDashboard.getBoolean(KEY + "Prepare", false);
    if (DriverStation.isDisabled()) {
      remoteAllowed = SmartDashboard.getBoolean(KEY + "RemoteAllowed", false);
    }
    if (DriverStation.isDisabled() && !prepare) {
      clearSteeringPasses();
    }
    // Preparation only changes controller limits while Driver Station is disabled. Apply the
    // limits even if Test has not been selected yet so disabled setup does not depend on a UI
    // mode toggle; actual diagnostic commands still require enabled Test mode below.
    boolean desiredLimits = prepare;
    if (DriverStation.isDisabled()) {
      if (desiredLimits != limitsRequested) {
        for (Module module : modules) {
          module.setDiagnosticLimits(desiredLimits);
        }
        limitsRequested = desiredLimits;
        clearSteeringPasses();
      }
    }
    boolean operatorAvailable = DriverStation.isDSAttached()
        && (remoteAllowed || DriverStation.isJoystickConnected(0));
    boolean testEnabled = DriverStation.isTestEnabled() && operatorAvailable;
    invalidateChangedGains();
    String fault = healthFault(Timer.getFPGATimestamp());
    if (!fault.isEmpty()) {
      clearSteeringPasses();
    }
    boolean ready = prepare && limitsRequested && fault.isEmpty() && timing.ready(nowSec);
    boolean continuous = Double.isFinite(nowSec)
        && (!Double.isFinite(lastNowSec)
            || (nowSec >= lastNowSec && nowSec - lastNowSec <= 0.1))
        && Double.isFinite(loopPeriodMs) && loopPeriodMs <= 100.0;
    lastNowSec = nowSec;

    synchronized (outputLock) {
      heartbeatSec = Timer.getFPGATimestamp();
      updateRemoteState(heartbeatSec);
      boolean held = remoteAllowed ? remoteActivation
          && RemoteDiagnosticPolicy.heartbeatFresh(heartbeatSec, remoteHeartbeatReceiptSec)
          : operatorAvailable && DriverStation.getStickButton(0, 1);
      boolean newRemoteRequest = remoteRequests.consume(
          SmartDashboard.getNumber(KEY + "StartNonce", 0),
          remoteAllowed && testEnabled && wasTestEnabled && held && ready && continuous && !active);
      wasTestEnabled = testEnabled;
    if (active && (!testEnabled || !held || !ready || !continuous)) {
      finish(false, !testEnabled ? "Test disabled or operator disconnected"
          : !held ? remoteAllowed ? "Remote activation off or heartbeat stale" : "Operator released Xbox A"
          : !continuous ? "Loop gap exceeds 100 ms"
          : !fault.isEmpty() ? fault : "Preparation or timing qualification lost");
    }
    boolean authorized;
    if (remoteAllowed) {
      gate.cancel();
      authorized = newRemoteRequest;
    } else {
      authorized = gate.update(testEnabled, held, ready && continuous);
    }
    if (!active && authorized) {
      activeRunNonce = remoteAllowed ? remoteRequests.lastConsumedNonce() : 0;
      start(nowSec);
    }
    if (active) {
      if (watchdogTripped && !"Deadline reached".equals(watchdogReason)) {
        finish(false, "Independent watchdog: " + watchdogReason);
      } else if (!selectionUnchanged()) {
        finish(false, "Diagnostic selection changed during movement");
      } else {
        runStage(nowSec, watchdogTripped);
      }
    }
    if (!active && testMode) {
      stopAll();
    }
    }
    if (!active && !ready && testMode) {
      status = !prepare ? "Blocked: Prepare must be set while Test mode is disabled"
          : !limitsRequested ? "Blocked: diagnostic limits must be applied while disabled"
          : !fault.isEmpty() ? "Blocked: " + fault
          : "Blocked: need 100 fresh Test loops, median <25 ms and p95 <40 ms";
    }
    Logger.recordOutput(KEY + "Ready", ready);
    Logger.recordOutput(KEY + "HealthFault", fault);
    Logger.recordOutput(KEY + "TimingQualified", timing.ready(nowSec));
    Logger.recordOutput(KEY + "Active", active);
    Logger.recordOutput(KEY + "Status", status);
    Logger.recordOutput(KEY + "StageActive", stage.name());
    Logger.recordOutput(KEY + "SelectedModule", selectedModule);
    Logger.recordOutput(KEY + "LoopMedianMs", timing.medianMs());
    Logger.recordOutput(KEY + "LoopP95Ms", timing.p95Ms());
    Logger.recordOutput(KEY + "LoopSamples", timing.sampleCount());
    Logger.recordOutput(KEY + "RequestedSpeedMetersPerSec", requestedSpeed);
    Logger.recordOutput(KEY + "TimestampSec", nowSec);
    Logger.recordOutput(KEY + "DSState", new double[] {Timer.getFPGATimestamp(),
        DriverStation.isEnabled() ? 1 : 0, DriverStation.isTest() ? 1 : 0,
        DriverStation.isDSAttached() ? 1 : 0, DriverStation.isEStopped() ? 1 : 0});
    Logger.recordOutput(KEY + "SteeringTargetsRad", targets);
    Logger.recordOutput(KEY + "SteeringPassed", allSteeringPassed());
    Logger.recordOutput(KEY + "WatchdogTripped", watchdogTripped);
    Logger.recordOutput(KEY + "WatchdogReason", watchdogReason);
    Logger.recordOutput(KEY + "RemoteAllowedLatched", remoteAllowed);
    Logger.recordOutput(KEY + "ActivationSource", remoteAllowed ? "Remote nonce + heartbeat" : "Physical Xbox A");
    Logger.recordOutput(KEY + "RemoteHeartbeatReceiptSec", remoteHeartbeatReceiptSec);
    Logger.recordOutput(KEY + "LastConsumedNonce", remoteRequests.lastConsumedNonce());
    Logger.recordOutput(KEY + "RemoteHeartbeatFresh", RemoteDiagnosticPolicy.heartbeatFresh(
        Timer.getFPGATimestamp(), remoteHeartbeatReceiptSec));
    SmartDashboard.putBoolean(KEY + "RemoteAllowedLatched", remoteAllowed);
    SmartDashboard.putNumber(KEY + "LastConsumedNonce", remoteRequests.lastConsumedNonce());
    SmartDashboard.putString(KEY + "ActivationSource", remoteAllowed ? "Remote nonce + heartbeat" : "Physical Xbox A");
    SmartDashboard.putString(KEY + "Status", status);
    SmartDashboard.putBoolean(KEY + "Ready", ready);
    SmartDashboard.putNumber(KEY + "LoopMedianMs", timing.medianMs());
    SmartDashboard.putNumber(KEY + "LoopP95Ms", timing.p95Ms());
    if (testMode || active) {
      logTrace(nowSec);
    }
  }

  private String healthFault(double nowSec) {
    for (int i = 0; i < modules.length; i++) {
      ModuleIO.ModuleIOInputs in = modules[i].getDiagnosticInputs();
      String prefix = "Module " + i + ": ";
      if (!DriveDiagnosticPolicy.sensorsHealthy(in.driveConnected, in.turnConnected,
          in.turnEncoderConnected, in.turnSeeded)) {
        return prefix + "sensor disconnected or steering not seeded";
      }
      if (!in.configurationHealthy || !in.diagnosticLimitsApplied) {
        return prefix + "configuration or diagnostic limits not verified";
      }
      if (!in.driveStatusHealthy || !in.turnStatusHealthy) {
        return prefix + "REV signal read failed";
      }
      if (!Double.isFinite(in.absoluteSensorAgeSec) || in.absoluteSensorAgeSec < 0
          || in.absoluteSensorAgeSec > 0.1
          || !DriveDiagnosticPolicy.samplesFresh(nowSec, in.snapshotTimestampSec)) {
        return prefix + "encoder telemetry stale";
      }
      if (!Double.isFinite(in.turnPosition.getRadians())
          || !Double.isFinite(in.turnAbsolutePosition.getRadians())
          || !Double.isFinite(in.driveVelocityRadPerSec)
          || !Double.isFinite(in.turnVelocityRadPerSec)) {
        return prefix + "invalid encoder reading";
      }
    }
    return "";
  }

  private void start(double nowSec) {
    Stage choice = selectedStage();
    double moduleChoice = SmartDashboard.getNumber(KEY + "Module", 0);
    if (choice == Stage.NONE || !Double.isFinite(moduleChoice)
        || moduleChoice != Math.rint(moduleChoice) || moduleChoice < 0 || moduleChoice > 4) {
      finish(false, "Choose a valid stage and module 0..3, or 4 for all");
      return;
    }
    selectedModule = (int) moduleChoice;
    if (selectedModule == 4 && choice != Stage.DRIVE_OPEN && choice != Stage.DRIVE_CLOSED) {
      finish(false, "Steering diagnostics require one module at a time");
      return;
    }
    if ((choice == Stage.STEER_POSITIVE || choice == Stage.STEER_NEGATIVE)
        && !feedbackPassed[selectedModule]) {
      finish(false, "A passing feedback-direction check is required before steering steps");
      return;
    }
    if (choice == Stage.DRIVE_CLOSED && (selectedModule != 4 || !allSteeringPassed())) {
      finish(false, "Closed-loop drive requires module 4 and all eight steering checks passed");
      return;
    }
    double requestedMagnitude = SmartDashboard.getNumber(KEY + "Magnitude", 0.5);
    if (choice == Stage.DRIVE_OPEN && !oneOf(requestedMagnitude, 0.5, 1.0, 1.5)
        || choice == Stage.DRIVE_CLOSED && !oneOf(requestedMagnitude, 0.25, 0.5)) {
      finish(false, "Magnitude must be one of the bounded stage presets");
      return;
    }
    for (int i = 0; i < modules.length; i++) {
      if (Math.abs(modules[i].getVelocityMetersPerSec()) > 0.1
          || Math.abs(modules[i].getDiagnosticInputs().turnVelocityRadPerSec) > 0.1) {
        finish(false, "All modules must be stationary before starting");
        return;
      }
      startingAngles[i] = modules[i].getAngle().getRadians();
      startingAbsoluteAngles[i] = modules[i].getDiagnosticInputs().turnAbsolutePosition.getRadians();
      targets[i] = startingAngles[i];
    }
    stage = choice;
    magnitude = requestedMagnitude;
    startSec = nowSec;
    returnPhase = false;
    requestedSpeed = 0;
    steadySpeeds.clear();
    steeringGuard.reset(nowSec, STEP_RAD);
    if (stage == Stage.FEEDBACK) {
      feedbackPassed[selectedModule] = false;
      Arrays.fill(steeringPassed[selectedModule], false);
    }
    if (stage == Stage.STEER_POSITIVE || stage == Stage.STEER_NEGATIVE) {
      int direction = stage == Stage.STEER_POSITIVE ? 0 : 1;
      steeringPassed[selectedModule][direction] = false;
      targets[selectedModule] = DriveDiagnosticPolicy.wrapRadians(startingAngles[selectedModule]
          + (stage == Stage.STEER_POSITIVE ? STEP_RAD : -STEP_RAD));
    }
    stopAll();
    active = true;
    watchdogTripped = false;
    watchdogReason = "";
    heartbeatSec = Timer.getFPGATimestamp();
    hardDeadlineSec = startSec + switch (stage) {
      case FEEDBACK -> 0.145;
      case DRIVE_OPEN -> 1.995;
      case DRIVE_CLOSED -> 2.995;
      case STEER_POSITIVE, STEER_NEGATIVE -> 11.995;
      default -> 0;
    };
    watchdogRunning = true;
    status = "Running " + stage.name() + " on module " + selectedModule;
  }

  private void runStage(double nowSec, boolean deadlineReached) {
    double elapsed = nowSec - startSec;
    if (!Double.isFinite(elapsed) || elapsed < 0) {
      finish(false, "Invalid diagnostic clock");
      return;
    }
    if (deadlineReached) {
      elapsed = switch (stage) {
        case FEEDBACK -> 0.150;
        case DRIVE_OPEN -> 2.0;
        case DRIVE_CLOSED -> 3.0;
        case STEER_POSITIVE, STEER_NEGATIVE -> 12.0;
        default -> elapsed;
      };
    }
    for (int i = 0; i < modules.length; i++) {
      if (!isSelected(i)) {
        modules[i].stop();
      }
    }
    switch (stage) {
      case FEEDBACK -> feedback(elapsed);
      case STEER_POSITIVE, STEER_NEGATIVE -> steeringStep(nowSec, elapsed);
      case DRIVE_OPEN -> driveOpen(nowSec, elapsed);
      case DRIVE_CLOSED -> driveClosed(nowSec, elapsed);
      default -> finish(false, "Invalid active stage");
    }
  }

  private void feedback(double elapsed) {
    double relative = angleTravel(selectedModule);
    double absolute = DriveDiagnosticPolicy.wrapRadians(
        modules[selectedModule].getDiagnosticInputs().turnAbsolutePosition.getRadians()
            - startingAbsoluteAngles[selectedModule]);
    if (DriveDiagnosticPolicy.steerPulseComplete(elapsed, relative)
        || Math.abs(absolute) >= TRAVEL_LIMIT_RAD) {
      boolean enough = Math.abs(relative) >= Math.toRadians(0.2)
          && Math.abs(absolute) >= Math.toRadians(0.2);
      boolean sameDirection = relative * absolute > 0;
      boolean agree = Math.abs(DriveDiagnosticPolicy.wrapRadians(relative - absolute))
          <= Math.toRadians(0.5);
      Logger.recordOutput(KEY + "FeedbackRelativeDeltaRad", relative);
      Logger.recordOutput(KEY + "FeedbackAbsoluteDeltaRad", absolute);
      feedbackPassed[selectedModule] = enough && sameDirection && agree;
      finish(enough && sameDirection && agree, !enough ? "Feedback motion below 0.2 degrees"
          : !sameDirection ? "Absolute and relative steering disagree in direction"
          : !agree ? "Absolute and relative steering disagree in magnitude"
          : "Steering feedback direction agrees");
      return;
    }
    commandOpen(selectedModule, 0, 0.25);
  }

  private void steeringStep(double nowSec, double elapsed) {
    if (elapsed >= 6 && !returnPhase) {
      targets[selectedModule] = startingAngles[selectedModule];
      returnPhase = true;
      steeringGuard.reset(nowSec, STEP_RAD);
    }
    double error = DriveDiagnosticPolicy.wrapRadians(
        targets[selectedModule] - modules[selectedModule].getAngle().getRadians());
    if (steeringGuard.update(nowSec, error)) {
      finish(false, "Steering error grew or repeatedly reversed");
      return;
    }
    double phaseElapsed = returnPhase ? elapsed - 6 : elapsed;
    if (phaseElapsed >= 1 && Math.abs(error) > HOLD_TOLERANCE_RAD) {
      finish(false, "Steering did not settle/hold within one degree");
      return;
    }
    if (elapsed >= 12) {
      steeringPassed[selectedModule][stage == Stage.STEER_POSITIVE ? 0 : 1] = true;
      finish(true, "Steering step and return both settled and held");
      return;
    }
    commandPosition(selectedModule, targets[selectedModule], 0);
  }

  private void driveOpen(double nowSec, double elapsed) {
    if (!driveTravelAndDirectionHealthy()) {
      return;
    }
    if (elapsed >= 2) {
      reportSpeeds(false);
      finish(true, "Bounded drive-voltage pulse completed; review recorded speeds/current");
      return;
    }
    for (int i = 0; i < modules.length; i++) {
      if (isSelected(i)) {
        commandOpen(i, magnitude, 0);
      }
    }
    collectSpeeds(nowSec);
  }

  private void driveClosed(double nowSec, double elapsed) {
    if (!driveTravelAndDirectionHealthy()) {
      return;
    }
    for (int i = 0; i < modules.length; i++) {
      if (Math.abs(DriveDiagnosticPolicy.wrapRadians(targets[i]
          - modules[i].getAngle().getRadians())) > TRAVEL_LIMIT_RAD) {
        finish(false, "Closed-loop steering heading error exceeds two degrees");
        return;
      }
    }
    if (elapsed >= 3) {
      boolean speedsPassed = reportSpeeds(true);
      finish(speedsPassed, speedsPassed ? "Drive speeds pass spread and target criteria"
          : "Drive speed spread or target tolerance failed");
      return;
    }
    requestedSpeed = magnitude * Math.min(1, elapsed);
    for (int i = 0; i < modules.length; i++) {
      commandPosition(i, targets[i], requestedSpeed);
    }
    if (elapsed >= 2) {
      collectSpeeds(nowSec);
    }
  }

  private boolean driveTravelAndDirectionHealthy() {
    for (int i = 0; i < modules.length; i++) {
      if (Math.abs(angleTravel(i)) > TRAVEL_LIMIT_RAD) {
        finish(false, "Unexpected steering travel exceeds two degrees on module " + i);
        return false;
      }
      if (isSelected(i) && modules[i].getVelocityMetersPerSec() < -0.03) {
        finish(false, "Drive encoder direction is negative on module " + i);
        return false;
      }
    }
    return true;
  }

  private void collectSpeeds(double nowSec) {
    double[] sample = new double[5];
    sample[0] = nowSec;
    for (int i = 0; i < modules.length; i++) {
      sample[i + 1] = modules[i].getVelocityMetersPerSec();
    }
    steadySpeeds.addLast(sample);
    while (!steadySpeeds.isEmpty() && nowSec - steadySpeeds.peekFirst()[0] > 1.0) {
      steadySpeeds.removeFirst();
    }
  }

  private boolean reportSpeeds(boolean requireTarget) {
    if (steadySpeeds.size() < 20
        || steadySpeeds.peekLast()[0] - steadySpeeds.peekFirst()[0] < 0.8) {
      Logger.recordOutput(KEY + "SpeedMetricsValid", false);
      return false;
    }
    double[] means = new double[4];
    for (double[] sample : steadySpeeds) {
      for (int i = 0; i < modules.length; i++) {
        means[i] += sample[i + 1] / steadySpeeds.size();
      }
    }
    double min = Double.POSITIVE_INFINITY;
    double max = Double.NEGATIVE_INFINITY;
    double sum = 0;
    int count = 0;
    boolean targetPass = true;
    for (int i = 0; i < modules.length; i++) {
      if (isSelected(i)) {
        min = Math.min(min, means[i]);
        max = Math.max(max, means[i]);
        sum += means[i];
        count++;
        targetPass &= Math.abs(means[i] - magnitude) <= magnitude * 0.1;
      }
    }
    double mean = sum / count;
    double spread = mean > 0.001 ? (max - min) / mean : Double.POSITIVE_INFINITY;
    Logger.recordOutput(KEY + "SpeedMetricsValid", true);
    Logger.recordOutput(KEY + "SteadySpeedMeansMetersPerSec", means);
    Logger.recordOutput(KEY + "SpeedSpreadFraction", spread);
    Logger.recordOutput(KEY + "SpeedTargetPass", requireTarget && targetPass);
    Logger.recordOutput(KEY + "SpeedSpreadPass", count == 4 && spread <= 0.05);
    return count == 4 && spread <= 0.05 && (!requireTarget || targetPass);
  }

  private void finish(boolean passed, String reason) {
    stopAll();
    active = false;
    watchdogRunning = false;
    requestedSpeed = 0;
    gate.cancel();
    status = (passed ? "PASS: " : "STOP: ") + reason
        + (remoteAllowed ? "; send a new StartNonce for another run" : "; release Xbox A before another run");
    Logger.recordOutput(KEY + "LastRunPassed", passed);
    Logger.recordOutput(KEY + "LastRunReason", reason);
    Logger.recordOutput(KEY + "LastRunNonce", activeRunNonce);
    Logger.recordOutput(KEY + "LastRunTimestampSec", Timer.getFPGATimestamp());
  }

  private boolean selectionUnchanged() {
    return selectedStage() == stage
        && SmartDashboard.getNumber(KEY + "Module", 0) == selectedModule
        && (stage != Stage.DRIVE_OPEN && stage != Stage.DRIVE_CLOSED
            || SmartDashboard.getNumber(KEY + "Magnitude", 0.5) == magnitude);
  }

  private Stage selectedStage() {
    try {
      return Stage.valueOf(SmartDashboard.getString(KEY + "Stage", "NONE").trim());
    } catch (IllegalArgumentException ex) {
      return Stage.NONE;
    }
  }

  private static boolean oneOf(double value, double... allowed) {
    return Double.isFinite(value)
        && Arrays.stream(allowed).anyMatch(candidate -> Math.abs(value - candidate) < 1e-9);
  }

  private boolean isSelected(int index) {
    return selectedModule == 4 || selectedModule == index;
  }

  private double angleTravel(int index) {
    return DriveDiagnosticPolicy.wrapRadians(modules[index].getAngle().getRadians()
        - startingAngles[index]);
  }

  private boolean allSteeringPassed() {
    for (boolean[] modulePass : steeringPassed) {
      if (!modulePass[0] || !modulePass[1]) {
        return false;
      }
    }
    return true;
  }

  private void clearSteeringPasses() {
    Arrays.fill(feedbackPassed, false);
    for (boolean[] modulePass : steeringPassed) {
      Arrays.fill(modulePass, false);
    }
  }

  private void invalidateChangedGains() {
    boolean changed = false;
    for (int i = 0; i < modules.length; i++) {
      ModuleIO.ModuleIOInputs in = modules[i].getDiagnosticInputs();
      double[] gains = {in.appliedDriveKp, in.appliedDriveKv, in.appliedTurnKp, in.appliedTurnKd};
      for (int gain = 0; gain < gains.length; gain++) {
        changed |= gainsObserved
            && Double.doubleToLongBits(verifiedGains[i][gain]) != Double.doubleToLongBits(gains[gain]);
        verifiedGains[i][gain] = gains[gain];
      }
      boolean[] state = {in.appliedDriveInverted, in.appliedTurnInverted,
          in.configurationHealthy, in.turnSeeded};
      for (int value = 0; value < state.length; value++) {
        changed |= gainsObserved && verifiedState[i][value] != state[value];
        verifiedState[i][value] = state[value];
      }
    }
    gainsObserved = true;
    if (changed) {
      clearSteeringPasses();
    }
  }

  private void stopAll() {
    for (Module module : modules) {
      module.stop();
    }
  }

  private void commandOpen(int index, double driveVolts, double turnVolts) {
    watchdogCheck();
    if (watchdogRunning && !watchdogTripped) {
      modules[index].runDiagnostic(driveVolts, turnVolts);
    }
  }

  private void commandPosition(int index, double angleRad, double speedMetersPerSec) {
    watchdogCheck();
    if (watchdogRunning && !watchdogTripped) {
      modules[index].runDiagnosticPosition(Rotation2d.fromRadians(angleRad), speedMetersPerSec);
    }
  }

  /** Read-only remote input polling under outputLock; repeated values do not refresh the lease. */
  private void updateRemoteState(double nowSec) {
    remoteActivation = SmartDashboard.getBoolean(KEY + "RemoteActivation", false);
    double value = SmartDashboard.getNumber(KEY + "RemoteHeartbeat", 0);
    if (Double.isFinite(value) && value > observedRemoteHeartbeat) {
      observedRemoteHeartbeat = value;
      remoteHeartbeatReceiptSec = nowSec;
    }
  }

  /** Independent stop path: never starts a stage or issues a nonzero reference. */
  private void watchdogCheck() {
    if (!watchdogRunning) {
      return;
    }
    synchronized (outputLock) {
      if (!watchdogRunning) {
        return;
      }
      double now = Timer.getFPGATimestamp();
      updateRemoteState(now);
      boolean activated = remoteAllowed ? remoteActivation
          && RemoteDiagnosticPolicy.heartbeatFresh(now, remoteHeartbeatReceiptSec)
          : DriverStation.isJoystickConnected(0) && DriverStation.getStickButton(0, 1);
      String reason = "";
      if (!DriverStation.isTestEnabled() || !DriverStation.isDSAttached()
          || !activated) {
        reason = "Operator activation or Test mode lost";
      } else if (now - heartbeatSec > 0.080) {
        reason = "Main-loop heartbeat stale";
      } else if (now >= hardDeadlineSec) {
        reason = "Deadline reached";
      }
      if (!reason.isEmpty()) {
        watchdogReason = reason;
        watchdogTripped = true;
        watchdogRunning = false;
        stopAll();
      }
    }
  }

  private void logTrace(double nowSec) {
    double[] trace = new double[4 + modules.length * 18];
    trace[0] = nowSec;
    trace[1] = stage.ordinal();
    trace[2] = selectedModule;
    trace[3] = requestedSpeed;
    for (int i = 0; i < modules.length; i++) {
      ModuleIO.ModuleIOInputs in = modules[i].getDiagnosticInputs();
      int base = 4 + i * 18;
      trace[base] = in.turnPosition.getRadians();
      trace[base + 1] = in.turnAbsolutePosition.getRadians();
      trace[base + 2] = targets[i];
      trace[base + 3] = DriveDiagnosticPolicy.wrapRadians(targets[i] - trace[base]);
      trace[base + 4] = modules[i].getVelocityMetersPerSec();
      trace[base + 5] = in.driveAppliedVolts;
      trace[base + 6] = in.turnAppliedVolts;
      trace[base + 7] = in.driveCurrentAmps;
      trace[base + 8] = in.turnCurrentAmps;
      trace[base + 9] = in.driveMotorRpm;
      trace[base + 10] = in.turnMotorRpm;
      trace[base + 11] = in.snapshotTimestampSec;
      trace[base + 12] = in.absoluteSensorAgeSec;
      trace[base + 13] = in.driveStatusHealthy ? 1 : 0;
      trace[base + 14] = in.turnStatusHealthy ? 1 : 0;
      trace[base + 15] = in.turnSeeded ? 1 : 0;
      trace[base + 16] = in.configurationHealthy ? 1 : 0;
      trace[base + 17] = in.diagnosticLimitsApplied ? 1 : 0;
    }
    Logger.recordOutput(KEY + "Trace", trace);
  }
}
