package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Four-motor flywheel plus a linear-actuator hood.
 *
 * <p>The hood is a two-position actuator pair, not a continuously variable surface, so range within
 * a hood position comes from flywheel RPM.
 */
public class Shooter extends VirtualSubsystem {
  private static final ShooterState kDefaultState = ShooterState.kIdle;

  private final AngularSubsystem flywheel;
  private final Hood hood;

  private ShooterState targetState = kDefaultState;

  public Shooter(AngularSubsystem flywheel, Hood hood) {
    this.flywheel = flywheel;
    this.hood = hood;

    flywheel.setDefaultCommand(flywheel.velocity(() -> targetState.getFlywheelVelocity()));
    // The hood is a two-position actuator with no feedback, so it is pushed rather than tracked.
    hood.setDefaultCommand(hood.run(() -> hood.setState(targetState.getHoodState())));
  }

  /** Holds {@code state} while scheduled, then falls back to idle. Bind with {@code whileTrue}. */
  public Command set(ShooterState state) {
    return Commands.startEnd(() -> targetState = state, () -> targetState = kDefaultState, this);
  }

  /** Latches {@code state} and finishes immediately. For auto sequences. */
  public Command setPersistent(ShooterState state) {
    return Commands.runOnce(() -> targetState = state);
  }

  public ShooterState getTargetState() {
    return targetState;
  }

  public void stopImmediately() {
    flywheel.stopImmediately();
    hood.stopImmediately();
  }

  /** Require all connected flywheel motors to reach the current state's target speed. */
  public Trigger atSpeed() {
    return new Trigger(
        () ->
            flywheel.areAllMotorsAtVelocity(
                targetState.getFlywheelVelocity(),
                ShooterConstants.kFlywheelSubsystemConfigReal.getVelocityTolerance()));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/TargetState", targetState.toString());
    Logger.recordOutput("Shooter/AtSpeed", atSpeed().getAsBoolean());
    Logger.recordOutput("Shooter/ReadyToFire", readyToFire().getAsBoolean());
  }

  /**
   * Hood is believed to have reached its commanded position. Timer-based, not measured — see {@link
   * Hood}.
   */
  public Trigger hoodAtTarget() {
    return hood.atPosition();
  }

  /** Both the flywheel and the hood are where the current state asks them to be. */
  public Trigger readyToFire() {
    return atSpeed().and(hoodAtTarget());
  }

  public HoodState getHoodState() {
    return hood.getTargetState();
  }

  /** Measured hood travel, 0.0 retracted to 1.0 extended. Zero if the potentiometer is absent. */
  public double getHoodMeasuredTravel() {
    return hood.getPositionNormalized();
  }

  /** Where the current state wants the hood: 0.0 retracted, 1.0 extended. */
  public double getHoodTargetTravel() {
    return targetState.getHoodState().isExtended() ? 1.0 : 0.0;
  }
}
