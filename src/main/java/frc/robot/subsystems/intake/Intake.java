package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystem;

/**
 * Pivoting arm plus the floor-pickup rollers and feeder it carries.
 *
 * <p><b>The arm and the rollers are commanded independently.</b> Holding intake runs the rollers
 * without moving the arm; a shot raises the arm without touching the rollers. Each has its own held
 * state and its own requirement token, so the two never interrupt one another.
 *
 * <p><b>The arm's resting position changes once per match.</b> It boots {@link
 * IntakePivotState#kStowed}, inside the frame. {@link #deploy()} puts it down and makes {@link
 * IntakePivotState#kDeployed} the resting position from then on, so releasing any button that moved
 * the arm sends it back down rather than back inside the frame.
 */
public class Intake extends VirtualSubsystem {
  private final AngularSubsystem pivot;
  private final AngularSubsystem intakeRollers;
  private final AngularSubsystem feederRollers;

  /** Requirement tokens, so pivot and roller commands arbitrate separately. */
  private final VirtualSubsystem pivotOwner = new VirtualSubsystem();

  private final VirtualSubsystem rollerOwner = new VirtualSubsystem();

  private IntakePivotState restingPivot = IntakePivotState.kStowed;
  private IntakePivotState heldPivot = null;
  private IntakeRollerState heldRollers = null;

  public Intake(
      AngularSubsystem pivot, AngularSubsystem intakeRollers, AngularSubsystem feederRollers) {
    this.pivot = pivot;
    this.intakeRollers = intakeRollers;
    this.feederRollers = feederRollers;

    pivot.setDefaultCommand(pivot.holdAtGoal(() -> getPivotState().getAngle()));
    intakeRollers.setDefaultCommand(
        intakeRollers.velocity(() -> getRollerState().getIntakeRollerVelocity()));
    feederRollers.setDefaultCommand(
        feederRollers.velocity(() -> getRollerState().getFeederVelocity()));
  }

  /** The arm position currently in force: whatever is being held, else the resting position. */
  public IntakePivotState getPivotState() {
    return heldPivot != null ? heldPivot : restingPivot;
  }

  public IntakeRollerState getRollerState() {
    return heldRollers != null ? heldRollers : IntakeRollerState.kOff;
  }

  /**
   * Holds the arm at {@code state} while scheduled, then returns it to its resting position — which
   * is down, once {@link #deploy()} has run. Bind with {@code whileTrue}, or add {@code
   * withTimeout} for a timed hold.
   */
  public Command setPivot(IntakePivotState state) {
    return Commands.startEnd(() -> heldPivot = state, () -> heldPivot = null, pivotOwner);
  }

  /** Runs the rollers at {@code state} while scheduled, then stops them. Does not move the arm. */
  public Command setRollers(IntakeRollerState state) {
    return Commands.startEnd(() -> heldRollers = state, () -> heldRollers = null, rollerOwner);
  }

  public boolean isDeployed() {
    return restingPivot == IntakePivotState.kDeployed;
  }

  /**
   * Puts the arm down for the match. Runs at the higher deploy current limit, because breaking the
   * hopper and intake free costs more than ordinary motion, then drops to the normal limit once the
   * arm arrives — so the high draw lasts a second or two rather than the whole match.
   *
   * <p>Waits on the measured angle rather than {@code atAngle()}: the latter reports against the
   * previous goal for a loop after the target changes, and would read "arrived" instantly.
   */
  public Command deploy() {
    return Commands.sequence(
        pivot.setCurrentLimit(IntakeConstants.kPivotDeployCurrentLimit),
        Commands.runOnce(() -> restingPivot = IntakePivotState.kDeployed),
        Commands.waitUntil(() -> isPivotNear(IntakePivotState.kDeployed))
            .withTimeout(IntakeConstants.kPivotDeployTimeout),
        pivot.setCurrentLimit(IntakeConstants.kPivotCurrentLimit));
  }

  /** {@link #deploy()}, but only if it has not already happened this match. */
  public Command deployOnce() {
    return Commands.either(Commands.none(), deploy(), this::isDeployed);
  }

  /** True when the measured arm angle is within tolerance of {@code state}. */
  public boolean isPivotNear(IntakePivotState state) {
    return Math.abs(pivot.getAngle().minus(state.getAngle()).in(Degrees))
        <= IntakeConstants.kPivotArrivalTolerance.in(Degrees);
  }

  public Trigger pivotNear(IntakePivotState state) {
    return new Trigger(() -> isPivotNear(state));
  }

  public Angle getMeasuredPivotAngle() {
    return pivot.getAngle();
  }

  public Angle getTargetPivotAngle() {
    return getPivotState().getAngle();
  }
}
