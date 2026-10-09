package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.subsystems.drive.DriveDiagnosticPolicy;
import frc.robot.subsystems.drive.RemoteDiagnosticPolicy;
import frc.robot.subsystems.intake.IntakeDiagnosticPolicy.Mode;
import frc.robot.subsystems.intake.IntakeDiagnosticPolicy.Reference;
import frc.robot.subsystems.intake.IntakeDiagnosticPolicy.Request;
import frc.robot.subsystems.intake.IntakeDiagnosticPolicy.Sample;
import org.littletonrobotics.junction.Logger;

/** Explicitly prepared, operator-held pivot-only Test diagnostics. Never returns/stows on stop. */
public final class IntakeDiagnostics implements AutoCloseable {
  private static final String KEY = "IntakeDiagnostics/";
  private final AngularSubsystem pivot;
  private final Object outputLock = new Object();
  private final Notifier watchdog;
  private final DriveDiagnosticPolicy.TimingWindow timing =
      new DriveDiagnosticPolicy.TimingWindow();
  private final DriveDiagnosticPolicy.DeadmanGate deadman = new DriveDiagnosticPolicy.DeadmanGate();
  private final RemoteDiagnosticPolicy.RequestGate nonces =
      new RemoteDiagnosticPolicy.RequestGate();
  private final RemoteDiagnosticPolicy.RequestGate calibrationNonces =
      new RemoteDiagnosticPolicy.RequestGate();
  private final RemoteDiagnosticPolicy.RequestGate directionNonces =
      new RemoteDiagnosticPolicy.RequestGate();
  private final IntakeDiagnosticPolicy.SelectionLatch selectionLatch =
      new IntakeDiagnosticPolicy.SelectionLatch();
  private final IntakeDiagnosticPolicy.StopLatch stopLatch = new IntakeDiagnosticPolicy.StopLatch();
  private final DriveDiagnosticPolicy.OscillationGuard oscillation =
      new DriveDiagnosticPolicy.OscillationGuard();
  private volatile boolean selected;
  private volatile boolean stopRequested;
  private volatile boolean remoteAllowed;
  private volatile boolean driveSelected;
  private volatile boolean running;
  private volatile String watchdogReason = "";
  private volatile double heartbeatSec;
  private volatile double deadlineSec;
  private double remoteHeartbeatObserved;
  private volatile double remoteHeartbeatReceiptSec = Double.NaN;
  private volatile boolean remoteActivation;
  private boolean calibrated;
  private boolean directionConfirmed;
  private boolean feedbackPassed;
  private boolean wasTest;
  private boolean wasEligible;
  private volatile boolean active;
  private Reference reference;
  private Request request;
  private Request preparedRequest;
  private double startSec;
  private double startRad;
  private double targetRad;
  private double settlingSince = Double.NaN;
  private double activeNonce;
  private double lastNowSec = Double.NaN;
  private String status = "Idle: disabled preparation and a physical reference are required";

  public IntakeDiagnostics(AngularSubsystem pivot) {
    this.pivot = pivot;
    SmartDashboard.putBoolean(KEY + "Prepare", false);
    SmartDashboard.putBoolean(KEY + "StopRequested", false);
    SmartDashboard.putBoolean(KEY + "ReferenceConfirmed", false);
    SmartDashboard.putBoolean(KEY + "ScalingAndTravelConfirmed", false);
    SmartDashboard.putBoolean(KEY + "DirectionConfirmed", false);
    SmartDashboard.putBoolean(KEY + "AllowRemote", false);
    SmartDashboard.putBoolean(KEY + "RemoteActivation", false);
    SmartDashboard.putString(KEY + "Mode", "OFF");
    SmartDashboard.putNumber(KEY + "ReferenceDegrees", Double.NaN);
    SmartDashboard.putNumber(KEY + "MinimumDegrees", Double.NaN);
    SmartDashboard.putNumber(KEY + "MaximumDegrees", Double.NaN);
    SmartDashboard.putNumber(KEY + "Voltage", 0.1);
    SmartDashboard.putNumber(KEY + "StepDegrees", 1.0);
    SmartDashboard.putNumber(KEY + "DurationSeconds", 0.1);
    SmartDashboard.putNumber(KEY + "OutputDuty", 0.03);
    SmartDashboard.putNumber(KEY + "CurrentLimitAmps", 10.0);
    SmartDashboard.putNumber(KEY + "TemperatureLimitCelsius", 45.0);
    SmartDashboard.putNumber(KEY + "RemoteHeartbeat", 0.0);
    SmartDashboard.putNumber(KEY + "StartNonce", 0.0);
    SmartDashboard.putNumber(KEY + "CalibrationNonce", 0.0);
    SmartDashboard.putNumber(KEY + "DirectionConfirmationNonce", 0.0);
    Logger.recordOutput(KEY + "LastRunPassed", false);
    Logger.recordOutput(KEY + "LastRunReason", "No completed run in this program instance");
    Logger.recordOutput(
        KEY + "TraceColumns",
        "time,mode,start,target,angle,velocity,volts,current,temperature,sampleTime,outputCap,nonce");
    watchdog = new Notifier(this::watchdogCheck);
    watchdog.setName("IntakePivotDiagnosticSafety");
    watchdog.startPeriodic(0.005);
  }

  /**
   * Selection latches only while disabled; clearing Prepare while enabled cannot restore defaults.
   */
  public boolean isSelected() {
    stopRequested =
        stopLatch.update(
            DriverStation.isDisabled(), SmartDashboard.getBoolean(KEY + "StopRequested", false));
    if (stopRequested) {
      selected = true;
      return true;
    }
    // Releasing an existing selection also requires a disabled acknowledged output-cap restore.
    boolean requested = SmartDashboard.getBoolean(KEY + "Prepare", false);
    if (requested || !selected)
      selected = selectionLatch.update(DriverStation.isDisabled(), requested);
    return selected;
  }

  /** Called in every mode; parent must inhibit other mechanisms and drive when selected. */
  public void periodic(double nowSec, double loopPeriodMs, boolean driveDiagnosticSelected) {
    driveSelected = driveDiagnosticSelected;
    boolean selection = isSelected();
    boolean test = DriverStation.isTest();
    if (test != wasTest) {
      timing.clear();
      wasTest = test;
    }
    if (test) timing.add(loopPeriodMs, nowSec);
    if (selection || test) pivot.diagnosticRefresh();
    boolean continuous =
        Double.isFinite(nowSec)
            && Double.isFinite(loopPeriodMs)
            && loopPeriodMs > 0
            && loopPeriodMs <= 80
            && (!Double.isFinite(lastNowSec) || nowSec > lastNowSec && nowSec - lastNowSec <= 0.08);
    lastNowSec = nowSec;
    heartbeatSec = Timer.getFPGATimestamp();
    updateRemote(heartbeatSec);
    boolean calibrate =
        calibrationNonces.consume(
            SmartDashboard.getNumber(KEY + "CalibrationNonce", 0),
            DriverStation.isDisabled() && selection && !driveSelected && !stopRequested);
    boolean confirmDirection =
        directionNonces.consume(
            SmartDashboard.getNumber(KEY + "DirectionConfirmationNonce", 0),
            DriverStation.isDisabled()
                && selection
                && calibrated
                && feedbackPassed
                && !driveSelected
                && !stopRequested);
    if (DriverStation.isDisabled()) prepareDisabled(selection, calibrate, confirmDirection);
    if (calibrated && !pivot.diagnosticInputs().referenceValid) {
      calibrated = false;
      directionConfirmed = false;
      feedbackPassed = false;
    }
    Request candidate = readRequest();
    String fault =
        !selection
            ? "Prepare must be selected while disabled"
            : stopRequested
                ? "Stop requested: isolation stays latched until explicitly cleared while disabled"
                : !SmartDashboard.getBoolean(KEY + "Prepare", false)
                    ? "Prepare cleared: isolation remains latched until disabled"
                    : driveSelected
                        ? "Drive and intake diagnostics are both selected"
                        : remoteAllowed != SmartDashboard.getBoolean(KEY + "AllowRemote", false)
                            ? "Remote permission changed: prepare while disabled"
                            : !calibrated
                                ? "Physical reference and measured travel have not been confirmed"
                                : !calibrationUnchanged()
                                    ? "Reference, travel or acknowledgements changed"
                                    : preparedRequest == null || !candidate.equals(preparedRequest)
                                        ? "Request changed: prepare while disabled"
                                        : !candidate.valid()
                                            ? "Request exceeds bounded diagnostic limits"
                                            : candidate.mode() == Mode.POSITION
                                                    && !directionConfirmed
                                                ? "Physical direction confirmation required for position steps"
                                                : IntakeDiagnosticPolicy.healthFault(
                                                    heartbeatSec, sample(), candidate);
    boolean ready = fault.isEmpty() && timing.ready(nowSec) && continuous;
    boolean eligible =
        DriverStation.isTestEnabled()
            && DriverStation.isDSAttached()
            && !DriverStation.isEStopped();
    boolean held = held(heartbeatSec);
    boolean remoteStart =
        nonces.consume(
            SmartDashboard.getNumber(KEY + "StartNonce", 0),
            remoteAllowed && eligible && wasEligible && ready && held && !active);
    wasEligible = eligible;
    if (active && (!eligible || !held || !ready || !selectionUnchanged())) {
      finish(
          false,
          !eligible
              ? "Test mode disabled or Driver Station lost"
              : !held
                  ? "Deadman released or remote heartbeat expired"
                  : !selectionUnchanged()
                      ? "Selection changed during movement"
                      : !fault.isEmpty() ? fault : "Timing qualification lost");
    }
    boolean physicalStart = false;
    if (remoteAllowed) deadman.cancel();
    else physicalStart = deadman.update(eligible, held, ready);
    if (!active && (remoteStart || physicalStart)) start(nowSec, candidate);
    if (active) run(nowSec);
    if (!active && (selection || test)) pivot.diagnosticStop();
    if (!active && !ready)
      status =
          "Blocked: "
              + (!fault.isEmpty()
                  ? fault
                  : "Need 100 fresh Test loops, median <25 ms and p95 <40 ms");
    log(nowSec, ready, fault);
  }

  private void prepareDisabled(boolean selection, boolean calibrate, boolean confirmDirection) {
    remoteAllowed = SmartDashboard.getBoolean(KEY + "AllowRemote", false);
    if (stopRequested) {
      pivot.diagnosticStop();
      return;
    }
    if (!SmartDashboard.getBoolean(KEY + "Prepare", false)) {
      if (selection) {
        pivot.diagnosticStop();
        if (!pivot.diagnosticSetOutputLimit(1.0)) {
          status = "Isolation retained: normal output cap restoration failed";
          return;
        }
        pivot.diagnosticRefresh();
        if (!pivot.diagnosticConfigReady()
            || Math.abs(pivot.diagnosticOutputLimit() - 1.0) > 1e-9) {
          status = "Isolation retained: normal output cap restoration not acknowledged";
          return;
        }
        selected = selectionLatch.update(true, false);
      }
      calibrated = false;
      preparedRequest = null;
      directionConfirmed = false;
      feedbackPassed = false;
      return;
    }
    pivot.diagnosticStop();
    Reference candidate = readReference();
    Request limits = readRequest();
    if (!SmartDashboard.getBoolean(KEY + "ReferenceConfirmed", false)
        || !SmartDashboard.getBoolean(KEY + "ScalingAndTravelConfirmed", false)
        || !candidate.valid()
        || !limits.valid()
        || driveSelected
        || !Double.isFinite(pivot.diagnosticMinimumAngleRadians())
        || !Double.isFinite(pivot.diagnosticMaximumAngleRadians())
        || candidate.minRad()
            < pivot.diagnosticMinimumAngleRadians() - IntakeDiagnosticPolicy.REFERENCE_ROUNDING_RAD
        || candidate.maxRad()
            > pivot.diagnosticMaximumAngleRadians()
                + IntakeDiagnosticPolicy.REFERENCE_ROUNDING_RAD) {
      calibrated = false;
      return;
    }
    if (reference != null && !reference.equals(candidate)) {
      calibrated = false;
      directionConfirmed = false;
      feedbackPassed = false;
    }
    if (calibrate) {
      reference = candidate;
      calibrated = pivot.diagnosticCalibrateReference(Radians.of(candidate.referenceRad()));
      feedbackPassed = false;
      directionConfirmed = false;
    }
    if (!SmartDashboard.getBoolean(KEY + "DirectionConfirmed", false)) directionConfirmed = false;
    if (confirmDirection && SmartDashboard.getBoolean(KEY + "DirectionConfirmed", false))
      directionConfirmed = true;
    if (calibrated && (preparedRequest == null || !preparedRequest.equals(limits))) {
      if (pivot.diagnosticSetOutputLimit(limits.outputDuty())) preparedRequest = limits;
      else preparedRequest = null;
    }
    // Explicit reference reset and controller writes must be visible in this loop's readiness/logs.
    if (calibrate || preparedRequest != null) pivot.diagnosticRefresh();
  }

  private void start(double nowSec, Request candidate) {
    activeNonce = remoteAllowed ? nonces.lastConsumedNonce() : 0;
    Sample current = sample();
    if (Math.abs(current.velocityRadPerSec()) > Math.toRadians(5)) {
      finish(false, "Pivot must be stationary before a diagnostic");
      return;
    }
    startRad = current.angleRad();
    targetRad = startRad + (candidate.mode() == Mode.POSITION ? candidate.stepRad() : 0);
    String travelFault =
        IntakeDiagnosticPolicy.motionFault(reference, candidate, startRad, targetRad, startRad);
    if (!travelFault.isEmpty()) {
      finish(false, travelFault);
      return;
    }
    request = candidate;
    if (candidate.mode() == Mode.FEEDBACK) {
      feedbackPassed = false;
      directionConfirmed = false;
    }
    startSec = nowSec;
    deadlineSec = Timer.getFPGATimestamp() + candidate.durationSec();
    settlingSince = Double.NaN;
    oscillation.reset(nowSec, candidate.stepRad());
    synchronized (outputLock) {
      pivot.diagnosticStop();
      watchdogReason = "";
      active = true;
      running = true;
    }
    status = "Running " + request.mode() + "; stop leaves the pivot at its present position";
  }

  private void run(double nowSec) {
    String motionFault =
        IntakeDiagnosticPolicy.motionFault(
            reference, request, startRad, targetRad, sample().angleRad());
    if (!motionFault.isEmpty()) {
      finish(false, motionFault);
      return;
    }
    if (!watchdogReason.isEmpty() && !watchdogReason.equals("Deadline reached")) {
      finish(false, "Independent watchdog: " + watchdogReason);
      return;
    }
    double elapsed = nowSec - startSec;
    if (!Double.isFinite(elapsed) || elapsed < 0) {
      finish(false, "Invalid monotonic clock");
      return;
    }
    if (request.mode() == Mode.POSITION) {
      double error = targetRad - pivot.getAngle().in(Radians);
      if (oscillation.update(nowSec, error)) {
        finish(false, "Position error grew or repeatedly reversed");
        return;
      }
      boolean settled =
          Math.abs(error) <= Math.toRadians(0.5)
              && Math.abs(pivot.getVelocity().in(RadiansPerSecond)) <= Math.toRadians(5);
      if (!settled) settlingSince = Double.NaN;
      else if (!Double.isFinite(settlingSince)) settlingSince = nowSec;
    }
    if (elapsed >= request.durationSec() || watchdogReason.equals("Deadline reached")) {
      boolean passed =
          request.mode() == Mode.FEEDBACK
              ? Math.abs(pivot.getAngle().in(Radians) - startRad) >= Math.toRadians(0.1)
              : Double.isFinite(settlingSince) && nowSec - settlingSince >= 0.15;
      if (request.mode() == Mode.FEEDBACK) feedbackPassed = passed;
      finish(
          passed,
          request.mode() == Mode.FEEDBACK
              ? "Feedback pulse completed; review measured direction/scale against physical motion"
              : passed
                  ? "Interior position step settled for 150 ms"
                  : "Position step did not settle for 150 ms");
      return;
    }
    watchdogCheck();
    // The lock covers only nonblocking motor commands; health reads/dashboard/logs remain outside.
    synchronized (outputLock) {
      if (!running) return;
      if (request.mode() == Mode.FEEDBACK) pivot.diagnosticSetOpenLoop(Volts.of(request.voltage()));
      else pivot.diagnosticSetAngle(Radians.of(targetRad));
    }
  }

  private void finish(boolean passed, String reason) {
    synchronized (outputLock) {
      running = false;
      active = false;
      pivot.diagnosticStop();
    }
    deadman.cancel();
    status = (passed ? "PASS: " : "STOP: ") + reason;
    Logger.recordOutput(KEY + "LastRunPassed", passed);
    Logger.recordOutput(KEY + "LastRunReason", reason);
    Logger.recordOutput(KEY + "LastRunNonce", activeNonce);
    Logger.recordOutput(KEY + "LastRunTimestampSec", Timer.getFPGATimestamp());
  }

  private void updateRemote(double nowSec) {
    remoteActivation = SmartDashboard.getBoolean(KEY + "RemoteActivation", false);
    double value = SmartDashboard.getNumber(KEY + "RemoteHeartbeat", 0);
    if (Double.isFinite(value)
        && value > remoteHeartbeatObserved
        && value <= 9_007_199_254_740_991.0
        && value == Math.rint(value)) {
      remoteHeartbeatObserved = value;
      remoteHeartbeatReceiptSec = nowSec;
    }
  }

  private boolean held(double nowSec) {
    return remoteAllowed
        ? SmartDashboard.getBoolean(KEY + "RemoteActivation", false)
            && RemoteDiagnosticPolicy.heartbeatFresh(nowSec, remoteHeartbeatReceiptSec)
        : DriverStation.isJoystickConnected(0) && DriverStation.getStickButton(0, 1);
  }

  private boolean calibrationUnchanged() {
    return reference != null
        && reference.equals(readReference())
        && SmartDashboard.getBoolean(KEY + "ReferenceConfirmed", false)
        && SmartDashboard.getBoolean(KEY + "ScalingAndTravelConfirmed", false)
        && (!directionConfirmed || SmartDashboard.getBoolean(KEY + "DirectionConfirmed", false));
  }

  private boolean selectionUnchanged() {
    return selected
        && SmartDashboard.getBoolean(KEY + "Prepare", false)
        && !driveSelected
        && request != null
        && request.equals(readRequest())
        && calibrationUnchanged()
        && remoteAllowed == SmartDashboard.getBoolean(KEY + "AllowRemote", false);
  }

  private void watchdogCheck() {
    if (!running) return;
    double now = Timer.getFPGATimestamp();
    boolean operator =
        remoteAllowed
            ? remoteActivation
                && RemoteDiagnosticPolicy.heartbeatFresh(now, remoteHeartbeatReceiptSec)
            : DriverStation.isJoystickConnected(0) && DriverStation.getStickButton(0, 1);
    // Deadline/heartbeat evaluation must not wait on NetworkTables, controller reads or logging.
    String fault =
        IntakeDiagnosticPolicy.watchdogFault(
            now,
            heartbeatSec,
            deadlineSec,
            DriverStation.isTestEnabled(),
            DriverStation.isDSAttached(),
            operator,
            !driveSelected && !stopRequested);
    if (fault.isEmpty()) return;
    synchronized (outputLock) {
      if (!running) return;
      watchdogReason = fault;
      running = false;
      pivot.diagnosticStop();
    }
  }

  private Reference readReference() {
    return new Reference(
        referenceRadians(SmartDashboard.getNumber(KEY + "ReferenceDegrees", Double.NaN)),
        referenceRadians(SmartDashboard.getNumber(KEY + "MinimumDegrees", Double.NaN)),
        referenceRadians(SmartDashboard.getNumber(KEY + "MaximumDegrees", Double.NaN)));
  }

  /** Use the same conversion as the typed physical limits, including an exact hard-stop seed. */
  static double referenceRadians(double degrees) {
    return Degrees.of(degrees).in(Radians);
  }

  private Request readRequest() {
    Mode mode;
    try {
      mode = Mode.valueOf(SmartDashboard.getString(KEY + "Mode", "OFF").trim());
    } catch (IllegalArgumentException e) {
      mode = Mode.OFF;
    }
    return new Request(
        mode,
        SmartDashboard.getNumber(KEY + "Voltage", 0.1),
        Math.toRadians(SmartDashboard.getNumber(KEY + "StepDegrees", 1)),
        SmartDashboard.getNumber(KEY + "DurationSeconds", 0.1),
        SmartDashboard.getNumber(KEY + "OutputDuty", 0.03),
        SmartDashboard.getNumber(KEY + "CurrentLimitAmps", 10),
        SmartDashboard.getNumber(KEY + "TemperatureLimitCelsius", 45));
  }

  private Sample sample() {
    return new Sample(
        pivot.diagnosticTimestampSeconds(),
        pivot.getAngle().in(Radians),
        pivot.getVelocity().in(RadiansPerSecond),
        pivot.diagnosticAppliedVolts().in(Volts),
        pivot.getStatorCurrent().in(Amps),
        pivot.diagnosticMaxTemperatureCelsius(),
        pivot.diagnosticConnected(),
        pivot.diagnosticConfigReady(),
        pivot.diagnosticOutputLimit(),
        pivot.diagnosticBusVolts().in(Volts));
  }

  private void log(double nowSec, boolean ready, String fault) {
    Sample s = sample();
    Logger.recordOutput(KEY + "Selected", selected);
    Logger.recordOutput(KEY + "StopRequestedLatched", stopRequested);
    Logger.recordOutput(KEY + "Ready", ready);
    Logger.recordOutput(KEY + "Active", active);
    Logger.recordOutput(KEY + "Calibrated", calibrated);
    Logger.recordOutput(KEY + "FeedbackPassed", feedbackPassed);
    Logger.recordOutput(KEY + "DirectionConfirmedLatched", directionConfirmed);
    Logger.recordOutput(KEY + "DirectionConfirmed", directionConfirmed);
    Logger.recordOutput(KEY + "ConfigurationReady", pivot.diagnosticConfigReady());
    Logger.recordOutput(KEY + "Status", status);
    Logger.recordOutput(KEY + "HealthFault", fault);
    Logger.recordOutput(KEY + "RemoteAllowedLatched", remoteAllowed);
    Logger.recordOutput(KEY + "LastConsumedNonce", nonces.lastConsumedNonce());
    Logger.recordOutput(KEY + "LastCalibrationNonce", calibrationNonces.lastConsumedNonce());
    Logger.recordOutput(
        KEY + "LastDirectionConfirmationNonce", directionNonces.lastConsumedNonce());
    Logger.recordOutput(KEY + "WatchdogReason", watchdogReason);
    Logger.recordOutput(KEY + "LoopMedianMs", timing.medianMs());
    Logger.recordOutput(KEY + "LoopP95Ms", timing.p95Ms());
    Logger.recordOutput(KEY + "LoopSamples", timing.sampleCount());
    var inputs = pivot.diagnosticInputs();
    Logger.recordOutput(
        KEY + "DSState",
        new double[] {
          nowSec,
          DriverStation.isEnabled() ? 1 : 0,
          DriverStation.isTest() ? 1 : 0,
          DriverStation.isDSAttached() ? 1 : 0,
          DriverStation.isEStopped() ? 1 : 0
        });
    Logger.recordOutput(
        KEY + "ConfigReadback",
        new double[] {
          inputs.readbackKP,
          inputs.readbackKI,
          inputs.readbackKD,
          inputs.readbackKS,
          inputs.readbackKV,
          inputs.readbackKG,
          inputs.readbackCruiseVelocityRadiansPerSecond,
          inputs.readbackAccelerationRadiansPerSecondPerSecond,
          inputs.readbackOutputLimit
        });
    Logger.recordOutput(
        KEY + "Trace",
        new double[] {
          nowSec,
          request == null ? 0 : request.mode().ordinal(),
          startRad,
          targetRad,
          s.angleRad(),
          s.velocityRadPerSec(),
          s.volts(),
          s.currentAmps(),
          s.temperatureC(),
          s.timestampSec(),
          s.outputDuty(),
          activeNonce
        });
    SmartDashboard.putString(KEY + "Status", status);
    SmartDashboard.putBoolean(KEY + "Ready", ready);
    SmartDashboard.putBoolean(KEY + "SelectedLatched", selected);
    SmartDashboard.putNumber(KEY + "LastConsumedNonce", nonces.lastConsumedNonce());
  }

  @Override
  public void close() {
    synchronized (outputLock) {
      running = false;
      active = false;
      pivot.diagnosticStop();
    }
    watchdog.close();
  }
}
