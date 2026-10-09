package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.lib.command.CachedTrigger;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivotState;
import frc.robot.subsystems.intake.IntakeRollerState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterState;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Cross-subsystem coordination. Individual subsystems own their own state machines; anything that
 * requires two or more of them to agree lives here.
 */
public class RobotSuperstructure {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  public RobotSuperstructure(Intake intake, Indexer indexer, Shooter shooter) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
  }

  /**
   * Run the arm's rollers and the indexer to pull FUEL in. <b>Does not move the arm</b> — it is
   * already down, having deployed during auto. Runs until cancelled.
   */
  public Command intakeFuel() {
    return intake
        .setRollers(IntakeRollerState.kIntaking)
        .alongWith(indexer.set(IndexerState.kIntaking));
  }

  /**
   * The full shot, held for as long as the trigger is:
   *
   * <ol>
   *   <li>Spin the flywheel up — started immediately, so it runs while the arm agitates.
   *   <li>Shuffle the arm slowly up and down a few times to settle FUEL toward the indexer.
   *   <li>Raise the arm 90 degrees and hold it there.
   *   <li>Feed once the flywheel is at speed, the hood is set, and the arm has actually arrived.
   * </ol>
   *
   * <p>Releasing the trigger cancels the whole thing, and because the arm's resting position is
   * down, it redeploys on its own — no explicit "lower the arm" step needed.
   */
  public Command shoot() {
    return shooter
        .set(ShooterState.kShootingNear)
        .alongWith(Commands.sequence(agitateArm(), raiseArmAndFeed()));
  }

  /**
   * A slow up/down shuffle of the arm. Built fresh on each call because a WPILib command instance
   * cannot appear in two compositions, so the steps cannot be shared between cycles.
   */
  private Command agitateArm() {
    int cycles = IntakeConstants.kAgitateCycles;
    double dwell = IntakeConstants.kAgitateDwell.in(edu.wpi.first.units.Units.Seconds);
    Command[] strokes = new Command[cycles * 2];
    for (int i = 0; i < cycles; i++) {
      strokes[2 * i] = intake.setPivot(IntakePivotState.kAgitateHigh).withTimeout(dwell);
      strokes[2 * i + 1] = intake.setPivot(IntakePivotState.kAgitateLow).withTimeout(dwell);
    }
    return Commands.sequence(strokes);
  }

  /** Raise and hold the arm, and start feeding only once everything is actually in position. */
  private Command raiseArmAndFeed() {
    return intake
        .setPivot(IntakePivotState.kRaised)
        .alongWith(
            Commands.waitUntil(
                    shooter.readyToFire().and(intake.pivotNear(IntakePivotState.kRaised)))
                .andThen(indexer.set(IndexerState.kFeeding)));
  }

  /** Puts the arm down once per match. Safe to call more than once. */
  public Command deployIntake() {
    return intake.deployOnce();
  }

  /**
   * Reverse the intake and the indexer — spits FUEL back out, and doubles as jam clearing since the
   * hopper is passive and the indexer is the only thing that can break up a pile.
   */
  public Command outtake() {
    return indexer
        .set(IndexerState.kUnjamming)
        .alongWith(intake.setRollers(IntakeRollerState.kEjecting));
  }

  /**
   * Registers everything PathPlanner/Choreo autos can reference by name. Called once from {@link
   * frc.robot.RobotContainer}'s constructor, before {@code AutoBuilder.buildAutoChooser()} reads
   * the deploy directory, so every name an {@code .auto} file mentions must be registered by the
   * time this returns.
   *
   * <p>Each command is {@code .asProxy()}-wrapped because it requires subsystems the path-following
   * command does not — without the proxy the scheduler cancels the path.
   */
  public void registerAutoCommands() {
    NamedCommands.registerCommand("Intake", intakeFuel().asProxy());
    // Agitation alone is kAgitateCycles x 2 x kAgitateDwell (2.1 s at the defaults), so the auto
    // shot needs room beyond that to raise the arm and actually feed. Revisit if those change.
    NamedCommands.registerCommand("Shoot", shoot().withTimeout(5.0).asProxy());
    NamedCommands.registerCommand("DeployIntake", deployIntake().asProxy());
    NamedCommands.registerCommand("SpinUp", shooter.setPersistent(ShooterState.kShootingNear));

    // Run the intake for a whole region of a path rather than at a single point.
    zoneTrigger("IntakeStart", "IntakeStop").whileTrue(intakeFuel().asProxy());
  }

  /**
   * Scales driver joystick input. Called three times per loop by the default drive command — twice
   * for translation (x and y) and once for rotation — so it must be cheap and side-effect free.
   *
   * @param rotation true when scaling the rotation axis (rad/s), false for translation (m/s)
   * @param turbo held to unlock full speed
   */
  public double getDriveMultiplier(boolean rotation, Trigger turbo) {
    if (turbo.getAsBoolean()) {
      return rotation ? DriveConstants.maxSpeedW() : DriveConstants.MAX_SPEED.get();
    }
    return rotation ? DriveConstants.transferSpeedW() : DriveConstants.TRANSFER_SPEED.get();
  }

  /**
   * A single command that exercises every mechanism through its range of motion, bound to an auto
   * so it can be run from the driver station during pit checks.
   */
  public Command fullRobotCheck() {
    return sequence(
            intake.setPivot(IntakePivotState.kDeployed).withTimeout(1.5),
            intake.setRollers(IntakeRollerState.kIntaking).withTimeout(1.0),
            intake.setPivot(IntakePivotState.kRaised).withTimeout(1.5),
            intake.setPivot(IntakePivotState.kStowed).withTimeout(1.5),
            indexer.set(IndexerState.kFeeding).withTimeout(1.0),
            indexer.set(IndexerState.kUnjamming).withTimeout(1.0),
            shooter.set(ShooterState.kShootingNear).withTimeout(2.5),
            shooter.set(ShooterState.kIdle).withTimeout(0.5))
        .asProxy();
  }

  /**
   * Builds a {@link Trigger} that latches true between two PathPlanner/Choreo named events. Use for
   * actions that should run for a region of a path rather than at a single point.
   *
   * <p>The returned trigger is a {@link CachedTrigger}, so it is polled once per loop rather than
   * once per binding — see {@link frc.robot.Robot#robotPeriodic()}.
   */
  private Trigger zoneTrigger(String startEvent, String stopEvent) {
    AtomicBoolean active = new AtomicBoolean(false);
    new EventTrigger(startEvent).onTrue(Commands.runOnce(() -> active.set(true)));
    new EventTrigger(stopEvent).onTrue(Commands.runOnce(() -> active.set(false)));
    return new CachedTrigger(active::get);
  }
}
