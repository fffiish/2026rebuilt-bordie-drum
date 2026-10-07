package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware boundary for the hood's pair of PA-14P linear actuators.
 *
 * <p>The actuators are 12 V brushed DC with a built-in, non-adjustable limit switch at each end of
 * travel — they cut themselves off at the stops, so no software current limit or timed cutoff is
 * needed. Drive them and let them finish.
 *
 * <p>The actuators fitted are plain two-wire PA-14s, so there is no position feedback and {@link
 * Hood} times the travel. The feedback path is still implemented: fit PA-14P units, wire their
 * potentiometers to analog inputs, and set {@code connected} true, and {@link Hood} starts
 * measuring instead of guessing without any other change.
 */
public interface HoodIO {
  @AutoLog
  class HoodIOInputs {
    /** What the last command asked for, not a measurement. */
    public boolean extendCommanded = false;

    /** Measured travel: 0.0 fully retracted, 1.0 fully extended. */
    public double positionNormalized = 0.0;

    /** Raw potentiometer reading, for calibrating the end points. */
    public double sensorVolts = 0.0;

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    /**
     * True only when a position sensor is present and trustworthy. The actuators fitted are plain
     * two-wire PA-14s with no potentiometer, so this is always false and {@link Hood} times the
     * travel instead. Fitting PA-14P units later makes it true with no other change.
     */
    public boolean connected = false;

    /** True when the motor controller itself is responding — unrelated to position feedback. */
    public boolean controllerConnected = false;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  /** Drive both actuators toward extended ({@code true}) or retracted ({@code false}). */
  default void setExtended(boolean extended) {}

  default void stop() {}
}
