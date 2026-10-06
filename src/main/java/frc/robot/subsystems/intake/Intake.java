package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystem;

/**
 * Pivoting arm carrying the feeder rollers.
 *
 * <p>This is a {@link VirtualSubsystem}: it owns no hardware itself, it owns a target {@link
 * IntakeState} and lets each mechanism's <em>default command</em> track it through suppliers. The
 * three {@link AngularSubsystem}s stay separate scheduler resources, so a command that needs only
 * the rollers does not block the pivot.
 *
 * <p>{@link #set} requires this virtual subsystem, which is what arbitrates between two bindings
 * both wanting a state.
 */
public class Intake extends VirtualSubsystem {
  private static final IntakeState kDefaultState = IntakeState.kStowed;

  private final AngularSubsystem pivot;
  private final AngularSubsystem intakeRollers;
  private final AngularSubsystem feederRollers;

  private IntakeState targetState = kDefaultState;

  public Intake(
      AngularSubsystem pivot, AngularSubsystem intakeRollers, AngularSubsystem feederRollers) {
    this.pivot = pivot;
    this.intakeRollers = intakeRollers;
    this.feederRollers = feederRollers;

    pivot.setDefaultCommand(pivot.holdAtGoal(() -> targetState.getPivotAngle()));
    intakeRollers.setDefaultCommand(
        intakeRollers.velocity(() -> targetState.getIntakeRollerVelocity()));
    feederRollers.setDefaultCommand(feederRollers.velocity(() -> targetState.getFeederVelocity()));
  }

  /**
   * Holds {@code state} for as long as the returned command is scheduled, then falls back to {@link
   * IntakeState#kStowed}. Bind with {@code whileTrue}.
   */
  public Command set(IntakeState state) {
    return Commands.startEnd(() -> targetState = state, () -> targetState = kDefaultState, this);
  }

  /** Latches {@code state} and finishes immediately. For auto sequences. */
  public Command setPersistent(IntakeState state) {
    return Commands.runOnce(() -> targetState = state);
  }

  public IntakeState getTargetState() {
    return targetState;
  }

  /** True once the arm has reached the angle its state asks for. */
  public Trigger atTarget() {
    return pivot.atAngle();
  }

  public Angle getMeasuredPivotAngle() {
    return pivot.getAngle();
  }

  public Angle getTargetPivotAngle() {
    return targetState.getPivotAngle();
  }
}
