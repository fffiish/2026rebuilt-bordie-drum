package frc.robot.subsystems.intake;

import frc.robot.subsystems.drive.DriveDiagnosticPolicy;
import frc.robot.subsystems.drive.RemoteDiagnosticPolicy;

/** Hardware-independent limits and checks for a physically referenced intake pivot. */
public final class IntakeDiagnosticPolicy {
  public static final double MAX_VOLTAGE = 0.25;
  public static final double MAX_FEEDBACK_SECONDS = 0.15;
  public static final double MAX_FEEDBACK_TRAVEL_RAD = Math.toRadians(2);
  public static final double MAX_STEP_RAD = Math.toRadians(2);
  public static final double MAX_POSITION_SECONDS = 1.0;
  public static final double MAX_OUTPUT_DUTY = 0.05;
  public static final double MAX_CURRENT_AMPS = 15.0;
  public static final double MAX_TEMPERATURE_C = 55.0;
  public static final double INTERIOR_MARGIN_RAD = Math.toRadians(5);
  public static final double REFERENCE_ROUNDING_RAD = 1e-6;

  private IntakeDiagnosticPolicy() {}

  public enum Mode {
    OFF,
    FEEDBACK,
    POSITION
  }

  /** Enabled dashboard edits cannot release subsystem isolation and restart default commands. */
  public static final class SelectionLatch {
    private boolean selected;

    public boolean update(boolean disabled, boolean requested) {
      if (disabled) selected = requested;
      return selected;
    }
  }

  /** A stop cannot be undone by an enabled dashboard edit or by disabling alone. */
  public static final class StopLatch {
    private boolean stopped;
    private boolean previousRequested;

    public boolean update(boolean disabled, boolean requested) {
      if (requested) stopped = true;
      else if (disabled && previousRequested) stopped = false;
      previousRequested = requested;
      return stopped;
    }
  }

  public record Reference(double referenceRad, double minRad, double maxRad) {
    public boolean valid() {
      return finite(referenceRad, minRad, maxRad)
          && minRad >= -Math.PI
          && maxRad <= Math.PI
          && maxRad - minRad >= Math.toRadians(2)
          && maxRad - minRad <= Math.toRadians(120)
          && referenceRad >= minRad
          && referenceRad <= maxRad;
    }

    public boolean interior(double angleRad) {
      return valid()
          && Double.isFinite(angleRad)
          && angleRad >= minRad + INTERIOR_MARGIN_RAD
          && angleRad <= maxRad - INTERIOR_MARGIN_RAD;
    }

    public boolean contains(double angleRad) {
      return valid()
          && Double.isFinite(angleRad)
          && angleRad >= minRad - REFERENCE_ROUNDING_RAD
          && angleRad <= maxRad + REFERENCE_ROUNDING_RAD;
    }

    /** Near either verified boundary a pulse can only move toward the interior. */
    public boolean feedbackStart(double angleRad, double voltage) {
      double margin = Math.min(INTERIOR_MARGIN_RAD, (maxRad - minRad) / 2.0);
      return contains(angleRad)
          && Double.isFinite(voltage)
          && voltage != 0
          && (angleRad >= minRad + margin || voltage > 0)
          && (angleRad <= maxRad - margin || voltage < 0);
    }
  }

  public record Request(
      Mode mode,
      double voltage,
      double stepRad,
      double durationSec,
      double outputDuty,
      double currentAmps,
      double temperatureC) {
    public boolean valid() {
      if (mode == null
          || mode == Mode.OFF
          || !finite(voltage, stepRad, durationSec, outputDuty, currentAmps, temperatureC)
          || outputDuty <= 0
          || outputDuty > MAX_OUTPUT_DUTY
          || currentAmps <= 0
          || currentAmps > MAX_CURRENT_AMPS
          || temperatureC <= 0
          || temperatureC > MAX_TEMPERATURE_C) return false;
      if (mode == Mode.FEEDBACK) {
        return Math.abs(voltage) >= 0.05
            && Math.abs(voltage) <= MAX_VOLTAGE
            && durationSec >= 0.02
            && durationSec <= MAX_FEEDBACK_SECONDS;
      }
      return Math.abs(stepRad) >= Math.toRadians(0.25)
          && Math.abs(stepRad) <= MAX_STEP_RAD
          && durationSec >= 0.3
          && durationSec <= MAX_POSITION_SECONDS;
    }
  }

  public record Sample(
      double timestampSec,
      double angleRad,
      double velocityRadPerSec,
      double volts,
      double currentAmps,
      double temperatureC,
      boolean connected,
      boolean configReady,
      double outputDuty,
      double busVolts) {
    public Sample(
        double timestampSec,
        double angleRad,
        double velocityRadPerSec,
        double volts,
        double currentAmps,
        double temperatureC,
        boolean connected,
        boolean configReady,
        double outputDuty) {
      this(
          timestampSec,
          angleRad,
          velocityRadPerSec,
          volts,
          currentAmps,
          temperatureC,
          connected,
          configReady,
          outputDuty,
          12.0);
    }
  }

  public static String healthFault(double nowSec, Sample sample, Request request) {
    if (!sample.connected()) return "Pivot disconnected or a REV read failed";
    if (!sample.configReady()) return "Controller configuration/readback not ready";
    if (!DriveDiagnosticPolicy.samplesFresh(nowSec, sample.timestampSec()))
      return "Pivot sample stale";
    if (!finite(
        sample.angleRad(),
        sample.velocityRadPerSec(),
        sample.volts(),
        sample.currentAmps(),
        sample.temperatureC(),
        sample.outputDuty(),
        sample.busVolts())) return "Invalid pivot sample";
    if (sample.busVolts() < 6 || sample.busVolts() > 16)
      return "Pivot bus voltage outside diagnostic range";
    if (sample.outputDuty() <= 0 || sample.outputDuty() > request.outputDuty() + 1e-9)
      return "Controller output cap not verified";
    if (Math.abs(sample.currentAmps()) >= request.currentAmps())
      return "Pivot current limit exceeded";
    if (sample.temperatureC() >= request.temperatureC()) return "Pivot temperature limit exceeded";
    if (Math.abs(sample.volts()) > sample.outputDuty() * sample.busVolts() + 0.05)
      return "Applied voltage exceeds verified output cap";
    if (request.mode() == Mode.FEEDBACK && Math.abs(sample.volts()) > MAX_VOLTAGE + 0.05)
      return "Applied feedback voltage exceeds pulse cap";
    return "";
  }

  public static String motionFault(
      Reference reference,
      Request request,
      double startingRad,
      double targetRad,
      double measuredRad) {
    if (!request.valid()) return "Invalid bounded motion request";
    double travel = measuredRad - startingRad;
    if (request.mode() == Mode.FEEDBACK) {
      if (!reference.feedbackStart(startingRad, request.voltage())
          || !reference.contains(measuredRad))
        return "Feedback must remain in verified travel and point inward near a boundary";
      if (Math.abs(travel) >= MAX_FEEDBACK_TRAVEL_RAD) return "Feedback travel limit reached";
      if (travel * request.voltage() < -Math.toRadians(0.2) * Math.abs(request.voltage()))
        return "Feedback moved opposite requested direction";
    } else {
      if (!reference.interior(startingRad) || !reference.interior(measuredRad))
        return "Pivot outside verified interior travel";
      if (!reference.interior(targetRad)) return "Position target outside verified interior travel";
      if (Math.abs(travel) > Math.abs(request.stepRad()) + Math.toRadians(1))
        return "Position travel exceeds step plus one degree";
      if ((targetRad - startingRad) * travel < -Math.toRadians(0.5) * Math.abs(request.stepRad()))
        return "Position moved opposite requested direction";
    }
    return "";
  }

  public static String watchdogFault(
      double nowSec,
      double mainHeartbeatSec,
      double deadlineSec,
      boolean testEnabled,
      boolean dsAttached,
      boolean held,
      boolean selectionUnchanged) {
    if (!testEnabled || !dsAttached || !held) return "Operator activation or Test mode lost";
    if (!selectionUnchanged) return "Diagnostic selection changed";
    if (!RemoteDiagnosticPolicy.heartbeatFresh(nowSec, mainHeartbeatSec, 0.080))
      return "Main-loop heartbeat stale";
    if (!Double.isFinite(deadlineSec) || nowSec >= deadlineSec) return "Deadline reached";
    return "";
  }

  private static boolean finite(double... values) {
    for (double value : values) if (!Double.isFinite(value)) return false;
    return true;
  }
}
