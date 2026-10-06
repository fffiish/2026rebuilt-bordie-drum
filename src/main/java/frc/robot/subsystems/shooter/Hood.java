package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.lib.subsystem.RegisteredSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Two-position hood driven by a pair of 12 V linear actuators.
 *
 * <p>This is deliberately <em>not</em> a {@link frc.robot.lib.subsystem.linear.LinearSubsystem}.
 * That class exists for closed-loop position control against an encoder; a bare 12 V actuator has
 * no encoder and only two useful states, so a PID loop would have nothing to close around.
 *
 * <p><b>"At position" is a timer, not a measurement.</b> With no feedback, the only thing the code
 * knows is how long ago it commanded a change. {@link #atPosition()} reports true once {@link
 * ShooterConstants#kHoodTravelTime} has elapsed since the last change — so the constant must be the
 * <em>worst-case</em> travel time, measured on the real mechanism, not the typical one. Add limit
 * switches to {@link HoodIO} if you want this to be real.
 */
public class Hood extends RegisteredSubsystem {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private HoodState targetState = HoodState.kNear;
  private final Timer sinceLastChange = new Timer();

  public Hood(HoodIO io) {
    this.io = io;
    sinceLastChange.start();
    io.setExtended(targetState.isExtended());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/TargetState", targetState.toString());
    Logger.recordOutput("Hood/AtPosition", isAtPosition());
    Logger.recordOutput("Hood/UsingSensor", inputs.connected);
  }

  /**
   * Commands {@code state} and holds it. Safe to call every loop; only a change restarts travel.
   */
  public void setState(HoodState state) {
    if (state != targetState) {
      targetState = state;
      sinceLastChange.restart();
      io.setExtended(state.isExtended());
    }
  }

  /** Command-factory form, for bindings and auto sequences. */
  public Command set(HoodState state) {
    return Commands.runOnce(() -> setState(state), this);
  }

  public HoodState getTargetState() {
    return targetState;
  }

  /** Estimated, not measured — see the class note. */
  public boolean isAtPosition() {
    return sinceLastChange.hasElapsed(
        ShooterConstants.kHoodTravelTime.in(edu.wpi.first.units.Units.Seconds));
  }

  public Trigger atPosition() {
    return new Trigger(this::isAtPosition);
  }
}
