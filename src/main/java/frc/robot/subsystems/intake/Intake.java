package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
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
 * without moving the arm; a shot agitates the arm without touching the rollers. Each has its own
 * held state and its own requirement token, so the two never interrupt one another.
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

  /** True while a deploy, retract or agitation owns the pivot's current limit. */
  private boolean pivotLimitBusy = false;

  /** True while the pivot is on the low rest limit. */
  private boolean restLimitActive = false;

  private IntakePivotState restingPivot = IntakePivotState.kStowed;
  private IntakePivotState heldPivot = null;

  /** While a slow raise runs, the arm aims here instead of straight at {@link #heldPivot}. */
  private Angle rampedPivotGoal = null;

  private IntakeRollerState heldRollers = null;
  private AngularVelocity heldFeederVelocity = null;
  private final VirtualSubsystem feederOwner = new VirtualSubsystem();

  public Intake(
      AngularSubsystem pivot, AngularSubsystem intakeRollers, AngularSubsystem feederRollers) {
    this.pivot = pivot;
    this.intakeRollers = intakeRollers;
    this.feederRollers = feederRollers;

    pivot.setDefaultCommand(
        pivot.holdAtGoal(
            () -> rampedPivotGoal != null ? rampedPivotGoal : getPivotState().getAngle()));
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
        () -> {
          heldFeederVelocity = IntakeConstants.kFeederIntaking;
          updateFeederCurrentLimit();
        },
        () -> {
          heldFeederVelocity = null;
          feederRollers.stopImmediately();
          updateFeederCurrentLimit();
        },
        feederOwner);
  }

  /**
   * TEMPORARY: the feeder runs at the low intaking limit only while intaking without a shot; a shot
   * (or anything else) gets the normal limit.
   */
  private void updateFeederCurrentLimit() {
    boolean intakingOnly = heldRollers == IntakeRollerState.kIntaking && heldFeederVelocity == null;
    feederRollers.applyCurrentLimit(
        intakingOnly
            ? IntakeConstants.kFeederIntakingCurrentLimit
            : IntakeConstants.kFeederCurrentLimit);
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

  /**
   * Sweeps the arm up and down between deployed and {@link IntakeConstants#kPivotAgitateHigh} for
   * as long as it is scheduled, each stroke taking {@code strokeTime}. Starts at deployed, moving
   * up. Runs at {@link IntakeConstants#kPivotAgitateCurrentLimit}; {@code finallyDo} restores the
   * normal limit and sends the arm back to its resting position on release.
   */
  public Command agitate(Time strokeTime) {
    Timer timer = new Timer();
    Angle low = IntakeConstants.kPivotDeployed;
    Angle high = IntakeConstants.kPivotAgitateHigh;
    return Commands.startRun(
            () -> {
              rampedPivotGoal = low;
              heldPivot = IntakePivotState.kAgitating;
              timer.restart();
              beginPivotLimit(IntakeConstants.kPivotAgitateCurrentLimit);
            },
            () -> {
              // Triangle wave: 0 -> 1 over one stroke, 1 -> 0 over the next, repeating.
              double phase = (timer.get() / strokeTime.in(Seconds)) % 2.0;
              double fraction = phase <= 1.0 ? phase : 2.0 - phase;
              rampedPivotGoal = low.plus(high.minus(low).times(fraction));
            },
            pivotOwner)
        .finallyDo(
            () -> {
              rampedPivotGoal = null;
              heldPivot = null;
              endPivotLimit();
            });
  }

  /** A command is taking over the pivot's current limit; the automatic rest limit stays out. */
  private void beginPivotLimit(Current limit) {
    pivotLimitBusy = true;
    restLimitActive = false;
    pivot.applyCurrentLimit(limit);
  }

  /** The command is done: back to the normal limit, and the rest limit may take over again. */
  private void endPivotLimit() {
    pivotLimitBusy = false;
    restLimitActive = false;
    pivot.applyCurrentLimit(IntakeConstants.kPivotCurrentLimit);
  }

  /**
   * While the arm sits near its deployed resting angle with nothing else moving it, drops the pivot
   * to {@link IntakeConstants#kPivotRestCurrentLimit} so a target below the physical stop does not
   * stall the motor. Backs off to the normal limit when the arm leaves (with hysteresis) or any
   * other pivot move starts.
   */
  private void updateRestLimit() {
    boolean resting =
        !pivotLimitBusy
            && heldPivot == null
            && rampedPivotGoal == null
            && restingPivot == IntakePivotState.kDeployed;
    double error =
        Math.abs(pivot.getAngle().minus(IntakePivotState.kDeployed.getAngle()).in(Degrees));
    double tolerance = IntakeConstants.kPivotArrivalTolerance.in(Degrees);
    if (!restLimitActive && resting && error <= tolerance) {
      restLimitActive = true;
      pivot.applyCurrentLimit(IntakeConstants.kPivotRestCurrentLimit);
    } else if (restLimitActive && !(resting && error <= 2.0 * tolerance)) {
      restLimitActive = false;
      pivot.applyCurrentLimit(IntakeConstants.kPivotCurrentLimit);
    }
  }

  /** Runs the rollers at {@code state} while scheduled, then stops them. Does not move the arm. */
  public Command setRollers(IntakeRollerState state) {
    return Commands.startEnd(
        () -> {
          heldRollers = state;
          updateFeederCurrentLimit();
          // Outtake gets a higher intake roller limit; everything else uses the normal one.
          intakeRollers.applyCurrentLimit(
              state == IntakeRollerState.kEjecting
                  ? IntakeConstants.kIntakeRollerEjectingCurrentLimit
                  : IntakeConstants.kIntakeRollerCurrentLimit);
        },
        () -> {
          heldRollers = null;
          intakeRollers.stopImmediately();
          // A shot may be feeding through these same rollers; releasing intake must not stop it.
          if (heldFeederVelocity == null) feederRollers.stopImmediately();
          updateFeederCurrentLimit();
          intakeRollers.applyCurrentLimit(IntakeConstants.kIntakeRollerCurrentLimit);
        },
        rollerOwner);
  }

  /** Latches the resting arm position without changing the independent roller state. */
  public Command setPersistentPivot(IntakePivotState state) {
    return Commands.runOnce(() -> restingPivot = state);
  }

  @Override
  public void periodic() {
    updateRestLimit();
    Logger.recordOutput("Intake/RestLimitActive", restLimitActive);
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
   * <p>This manual deploy (X) runs at {@link IntakeConstants#kPivotManualDeployCurrentLimit} (65
   * A). The first, automatic deploy ({@link #deployOnce()}) runs at {@link
   * IntakeConstants#kPivotDeployCurrentLimit} (80 A). Breaking the hopper and intake free costs
   * more than ordinary motion. Once the arm arrives — or after {@link
   * IntakeConstants#kPivotDeployTimeout} — the limit is back at the normal 40 A, which is what
   * shooting and raising run at.
   *
   * <p>The drop back is in {@code finallyDo}, so it happens even if the deploy is interrupted or
   * the robot is disabled mid-deploy. Without that, an interrupted deploy would leave the pivot at
   * 80 A for the rest of the match.
   *
   * <p>Waits on the measured angle rather than {@code atAngle()}: the latter reports against the
   * previous goal for a loop after the target changes, and would read "arrived" instantly.
   */
  public Command deploy() {
    return deploy(IntakeConstants.kPivotManualDeployCurrentLimit);
  }

  /** {@link #deploy()} at {@code limit} until the arm arrives or the timeout passes. */
  private Command deploy(Current limit) {
    return Commands.sequence(
            Commands.runOnce(() -> beginPivotLimit(limit)),
            Commands.runOnce(() -> restingPivot = IntakePivotState.kDeployed),
            Commands.waitUntil(() -> isPivotNear(IntakePivotState.kDeployed))
                .withTimeout(IntakeConstants.kPivotDeployTimeout))
        .finallyDo(this::endPivotLimit);
  }

  /**
   * Folds the arm back to stowed and makes stowed its resting position, the reverse of {@link
   * #deploy()}. Runs at {@link IntakeConstants#kPivotRetractCurrentLimit} (60 A) until the arm
   * arrives or {@link IntakeConstants#kPivotRetractTimeout} passes, then drops back to {@link
   * IntakeConstants#kPivotCurrentLimit}; {@code finallyDo} covers interruption. X deploys it again.
   */
  public Command retract() {
    return Commands.sequence(
            Commands.runOnce(() -> beginPivotLimit(IntakeConstants.kPivotRetractCurrentLimit)),
            Commands.runOnce(() -> restingPivot = IntakePivotState.kStowed),
            Commands.waitUntil(() -> isPivotNear(IntakePivotState.kStowed))
                .withTimeout(IntakeConstants.kPivotRetractTimeout))
        .finallyDo(this::endPivotLimit);
  }

  /**
   * Declares that the arm is <em>down right now</em>, for a robot that was powered on with the arm
   * out instead of folded.
   *
   * <p>The arm has no absolute encoder, only the motor's relative one, so it cannot tell where it
   * is at power-on. It assumes folded and zeroes there. If it was actually down, every later
   * command aims from the wrong starting point and drives the arm toward the floor. This re-zeroes
   * the encoder to the deployed angle and makes deployed the resting position, so the arm stays
   * where it is and raising and redeploying all work from the right reference.
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

  /**
   * The automatic deploy: only if it has not already happened this match, and the one deploy that
   * runs at {@link IntakeConstants#kPivotDeployCurrentLimit} (80 A) to break the hopper and intake
   * free. Everything after, including X, runs at the normal {@link
   * IntakeConstants#kPivotCurrentLimit}.
   */
  public Command deployOnce() {
    return Commands.either(
        Commands.none(), deploy(IntakeConstants.kPivotDeployCurrentLimit), this::isDeployed);
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
    return rampedPivotGoal != null ? rampedPivotGoal : getPivotState().getAngle();
  }
}
