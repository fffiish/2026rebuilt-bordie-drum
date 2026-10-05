package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.lib.subsystem.linear.LinearSubsystem;

/**
 * Two-motor flywheel plus a linear-actuator hood.
 *
 * <p>The hood is a {@link LinearSubsystem} tracked in inches of extension, not degrees of launch
 * angle — see {@link frc.robot.constants.shooter.ShooterConstants}.
 */
public class Shooter extends VirtualSubsystem {
  private static final ShooterState kDefaultState = ShooterState.kIdle;

  private final AngularSubsystem flywheel;
  private final LinearSubsystem hood;

  private ShooterState targetState = kDefaultState;

  public Shooter(AngularSubsystem flywheel, LinearSubsystem hood) {
    this.flywheel = flywheel;
    this.hood = hood;

    flywheel.setDefaultCommand(flywheel.velocity(() -> targetState.getFlywheelVelocity()));
    hood.setDefaultCommand(hood.holdAtGoal(() -> targetState.getHoodLength()));
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

  /**
   * Flywheel is within its velocity tolerance of the commanded speed. {@code atAngle()} is
   * mode-aware — in velocity mode it compares goal velocity to measured velocity — so this is the
   * correct "spun up" signal, despite the name.
   */
  public Trigger atSpeed() {
    return flywheel.atAngle();
  }

  /** Hood has reached its commanded extension. */
  public Trigger hoodAtTarget() {
    return hood.atLength();
  }

  /** Both the flywheel and the hood are where the current state asks them to be. */
  public Trigger readyToFire() {
    return atSpeed().and(hoodAtTarget());
  }

  public Distance getMeasuredHoodLength() {
    return hood.getLength();
  }

  public Distance getTargetHoodLength() {
    return targetState.getHoodLength();
  }
}
