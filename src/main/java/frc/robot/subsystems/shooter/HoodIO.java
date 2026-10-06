package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware boundary for the hood's pair of PA-14P linear actuators.
 *
 * <p>The actuators are 12 V brushed DC with a built-in, non-adjustable limit switch at each end of
 * travel — they cut themselves off at the stops, so no software current limit or timed cutoff is
 * needed. Drive them and let them finish.
 *
 * <p>The P variant carries a potentiometer, so {@link HoodIOInputs#positionNormalized} is a real
 * measurement rather than an assumption. {@link Hood} prefers it and falls back to a timer only if
 * the sensor reads disconnected, which means a failed potentiometer degrades the robot to the
 * previous behaviour instead of freezing the hood.
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

    /** False when the potentiometer looks unplugged — Hood falls back to timing. */
    public boolean connected = false;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  /** Drive both actuators toward extended ({@code true}) or retracted ({@code false}). */
  default void setExtended(boolean extended) {}

  default void stop() {}
}
