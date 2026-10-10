package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import org.littletonrobotics.junction.Logger;

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
  private AngularVelocity heldFeederVelocity = null;
  private final VirtualSubsystem feederOwner = new VirtualSubsystem();

  public Intake(
      AngularSubsystem pivot, AngularSubsystem intakeRollers, AngularSubsystem feederRollers) {
    this.pivot = pivot;
    this.intakeRollers = intakeRollers;
    this.feederRollers = feederRollers;

    pivot.setDefaultCommand(pivot.holdAtGoal(() -> getPivotState().getAngle()));
    intakeRollers.setDefaultCommand(
        intakeRollers.velocity(() -> getRollerState().getIntakeRollerVelocity()));
    feederRollers.setDefaultCommand(
        feederRollers.velocity(
            () ->
                heldFeederVelocity != null
                    ? heldFeederVelocity
                    : getRollerState().getFeederVelocity()));
  }

  /** Feed during a shot without starting the independent floor pickup rollers. */
  public Command feedShooter() {
    return Commands.startEnd(
        () -> heldFeederVelocity = IntakeConstants.kFeederIntaking,
        () -> {
          heldFeederVelocity = null;
          feederRollers.stopImmediately();
        },
        feederOwner);
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
    return Commands.startEnd(
        () -> heldRollers = state,
        () -> {
          heldRollers = null;
          intakeRollers.stopImmediately();
          // A shot may be feeding through these same rollers; releasing intake must not stop it.
          if (heldFeederVelocity == null) feederRollers.stopImmediately();
        },
        rollerOwner);
  }

  /** Latches the resting arm position without changing the independent roller state. */
  public Command setPersistentPivot(IntakePivotState state) {
    return Commands.runOnce(() -> restingPivot = state);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/TargetState", getPivotState().name());
    Logger.recordOutput("Intake/TargetPivotState", getPivotState().name());
    Logger.recordOutput("Intake/TargetRollerState", getRollerState().name());
    Logger.recordOutput(
        "Intake/FeederTargetRps",
        (heldFeederVelocity != null ? heldFeederVelocity : getRollerState().getFeederVelocity())
            .in(RotationsPerSecond));
    Logger.recordOutput("Intake/TargetPivotDegrees", getTargetPivotAngle().in(Degrees));
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

  public boolean isDeployed() {
    return restingPivot == IntakePivotState.kDeployed;
  }

  /**
   * Puts the arm down for the match and makes down its resting position.
   *
   * <p>Deploying runs at {@link IntakeConstants#kPivotDeployCurrentLimit} (80 A), because breaking
   * the hopper and intake free costs more than ordinary motion. Once the arm arrives — or after
   * {@link IntakeConstants#kPivotDeployTimeout} — it drops back to {@link
   * IntakeConstants#kPivotCurrentLimit} (40 A), which is what shooting and agitating run at.
   *
   * <p>The drop back is in {@code finallyDo}, so it happens even if the deploy is interrupted or
   * the robot is disabled mid-deploy. Without that, an interrupted deploy would leave the pivot at
   * 80 A for the rest of the match.
   *
   * <p>Waits on the measured angle rather than {@code atAngle()}: the latter reports against the
   * previous goal for a loop after the target changes, and would read "arrived" instantly.
   */
  public Command deploy() {
    return Commands.sequence(
            Commands.runOnce(
                () -> pivot.applyCurrentLimit(IntakeConstants.kPivotDeployCurrentLimit)),
            Commands.runOnce(() -> restingPivot = IntakePivotState.kDeployed),
            Commands.waitUntil(() -> isPivotNear(IntakePivotState.kDeployed))
                .withTimeout(IntakeConstants.kPivotDeployTimeout))
        .finallyDo(() -> pivot.applyCurrentLimit(IntakeConstants.kPivotCurrentLimit));
  }

  /**
   * Folds the arm back to stowed and makes stowed its resting position, the reverse of {@link
   * #deploy()}. Runs at {@link IntakeConstants#kPivotRetractCurrentLimit} (60 A) until the arm
   * arrives or {@link IntakeConstants#kPivotRetractTimeout} passes, then drops back to {@link
   * IntakeConstants#kPivotCurrentLimit}; {@code finallyDo} covers interruption. X deploys it again.
   */
  public Command retract() {
    return Commands.sequence(
            Commands.runOnce(
                () -> pivot.applyCurrentLimit(IntakeConstants.kPivotRetractCurrentLimit)),
            Commands.runOnce(() -> restingPivot = IntakePivotState.kStowed),
            Commands.waitUntil(() -> isPivotNear(IntakePivotState.kStowed))
                .withTimeout(IntakeConstants.kPivotRetractTimeout))
        .finallyDo(() -> pivot.applyCurrentLimit(IntakeConstants.kPivotCurrentLimit));
  }

  /**
   * Declares that the arm is <em>down right now</em>, for a robot that was powered on with the arm
   * out instead of folded.
   *
   * <p>The arm has no absolute encoder, only the motor's relative one, so it cannot tell where it
   * is at power-on. It assumes folded and zeroes there. If it was actually down, every later
   * command aims from the wrong starting point and drives the arm toward the floor. This re-zeroes
   * the encoder to the deployed angle and makes deployed the resting position, so the arm stays
   * where it is and agitation, raising and redeploying all work from the right reference.
   *
   * <p>Runs while disabled, so it can be pressed before enabling. Press it once, only when the arm
   * really is down.
   */
  public Command markArmDeployed() {
    return Commands.sequence(
            Commands.runOnce(() -> restingPivot = IntakePivotState.kDeployed),
            pivot.resetAngle(IntakeConstants.kPivotDeployed))
        .ignoringDisable(true);
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
