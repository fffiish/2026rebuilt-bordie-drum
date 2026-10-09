package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
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
 * <p>The PA-14P's potentiometer makes {@link #atPosition()} a real measurement. The timer path
 * survives only as a fallback for a disconnected sensor, so {@link
 * ShooterConstants#kHoodTravelTime} still wants to be a worst-case number.
 */
public class Hood extends RegisteredSubsystem {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private HoodState targetState = HoodState.kNear;
  private final Timer sinceLastChange = new Timer();

  private final Alert controllerDisconnectedAlert =
      new Alert("Hood motor controller disconnected!", Alert.AlertType.kError);

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
    controllerDisconnectedAlert.set(!inputs.controllerConnected);
    Logger.recordOutput("Hood/UsingSensor", inputs.connected);
    controllerDisconnectedAlert.set(!inputs.controllerConnected);
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

  public void stopImmediately() { io.stop(); }

  /**
   * True once the hood has reached the position its state asks for.
   *
   * <p>Prefers the potentiometer: compares measured travel against the target end point within
   * {@link ShooterConstants#kHoodPositionTolerance}. If the sensor reads disconnected it falls back
   * to {@link ShooterConstants#kHoodTravelTime} elapsing, so a dead potentiometer costs accuracy
   * rather than the use of the mechanism.
   */
  public boolean isAtPosition() {
    if (inputs.connected) {
      double target = targetState.isExtended() ? 1.0 : 0.0;
      return Math.abs(inputs.positionNormalized - target)
          <= ShooterConstants.kHoodPositionTolerance;
    }
    return sinceLastChange.hasElapsed(
        ShooterConstants.kHoodTravelTime.in(edu.wpi.first.units.Units.Seconds));
  }

  /** Measured travel, 0.0 retracted to 1.0 extended. Meaningless if the sensor is disconnected. */
  public double getPositionNormalized() {
    return inputs.positionNormalized;
  }

  /** False when the potentiometer is not reporting — atPosition() is then a timer, not a fact. */
  public boolean isSensorConnected() {
    return inputs.connected;
  }

  public Trigger atPosition() {
    return new Trigger(this::isAtPosition);
  }
}
