package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import org.littletonrobotics.junction.Logger;

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
  private boolean rollersStoppedByRelease;

  public Intake(
      AngularSubsystem pivot, AngularSubsystem intakeRollers, AngularSubsystem feederRollers) {
    this.pivot = pivot;
    this.intakeRollers = intakeRollers;
    this.feederRollers = feederRollers;

    pivot.setDefaultCommand(pivot.holdAtGoal(() -> targetState.getPivotAngle()));
    intakeRollers.setDefaultCommand(
        intakeRollers.velocity(
            () ->
                rollersStoppedByRelease
                    ? RotationsPerSecond.of(0)
                    : targetState.getIntakeRollerVelocity()));
    feederRollers.setDefaultCommand(
        feederRollers.velocity(
            () ->
                rollersStoppedByRelease
                    ? RotationsPerSecond.of(0)
                    : targetState.getFeederVelocity()));
  }

  /**
   * Holds {@code state} for as long as the returned command is scheduled, then falls back to {@link
   * IntakeState#kStowed}. Bind with {@code whileTrue}.
   */
  public Command set(IntakeState state) {
    return Commands.startEnd(
        () -> {
          rollersStoppedByRelease = false;
          targetState = state;
        },
        () -> {
          targetState = kDefaultState;
          intakeRollers.stopImmediately();
          feederRollers.stopImmediately();
        },
        this);
  }

  /** Run pickup and feeder rollers while held, leaving the pivot's target and command untouched. */
  public Command runRollers() {
    return intakeRollers
        .velocity(IntakeConstants.kIntakeRollerIntaking)
        .alongWith(feederRollers.velocity(IntakeConstants.kFeederIntaking))
        .beforeStarting(() -> rollersStoppedByRelease = false)
        .finallyDo(
            () -> {
              rollersStoppedByRelease = true;
              intakeRollers.stopImmediately();
              feederRollers.stopImmediately();
            });
  }

  /** Latches {@code state} and finishes immediately. For auto sequences. */
  public Command setPersistent(IntakeState state) {
    return Commands.runOnce(
        () -> {
          rollersStoppedByRelease = false;
          targetState = state;
        });
  }

  public IntakeState getTargetState() {
    return targetState;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/TargetState", targetState.name());
    Logger.recordOutput("Intake/TargetPivotDegrees", targetState.getPivotAngle().in(Degrees));
  }

  public void stopImmediately() {
    pivot.stopImmediately();
    intakeRollers.stopImmediately();
    feederRollers.stopImmediately();
  }

  /** Access for the isolated diagnostic path; normal commands continue to own this resource. */
  public AngularSubsystem getDiagnosticPivot() {
    return pivot;
  }

  /** Refresh and stop both roller groups without polling bindings or scheduling defaults. */
  public void diagnosticStopRollers() {
    intakeRollers.diagnosticStop();
    feederRollers.diagnosticStop();
    intakeRollers.diagnosticRefresh();
    feederRollers.diagnosticRefresh();
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
