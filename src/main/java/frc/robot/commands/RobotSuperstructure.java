package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
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
import java.util.function.BooleanSupplier;

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
   * Run the arm's pickup and feeder rollers to pull FUEL in. Does not command the indexer. <b>Does
   * not move the arm</b> — it is already down, having deployed during auto. Runs until cancelled.
   */
  public Command intakeFuel() {
    return intake.setRollers(IntakeRollerState.kIntaking);
  }

  /** Driver pickup rollers while held; releasing stops them without moving the pivot. */
  public void bindIntakeTrigger(Trigger intakeTrigger) {
    intakeTrigger.whileTrue(intakeFuel());
  }

  public Command intakeFuelWithoutDeploy() {
    return intakeFuel();
  }

  /** Auto deploys once, and teleop also deploys for practice without an autonomous period. */
  public void bindDeploymentTriggers() {
    RobotModeTriggers.autonomous().onTrue(deployIntake());
    RobotModeTriggers.teleop().onTrue(deployIntake());
  }

  /** Explicit deployment for enabled Test with the existing diagnostic selection respected. */
  public Command deployIntakeForTest(BooleanSupplier diagnosticSelected) {
    return deployIntake()
        .onlyIf(() -> DriverStation.isTestEnabled() && !diagnosticSelected.getAsBoolean());
  }

  /**
   * The full shot, held for as long as the trigger is. Two things run side by side from the moment
   * it is pressed:
   *
   * <ul>
   *   <li><b>Spin-up.</b> The flywheel immediately runs at its open-loop shooting voltage.
   *   <li><b>The shot.</b> Once flywheel speed is within tolerance, the bottom rollers and feeder
   *       start together. The arm shuffles to shake FUEL toward the indexer, then raises and holds.
   * </ul>
   *
   * <p>Releasing the trigger cancels both, and because the arm's resting position is down, it
   * redeploys on its own.
   */
  public Command shoot() {
    return shooter
        .set(ShooterState.kShootingNear)
        .alongWith(
            Commands.waitUntil(
                    () -> !Constants.shooterHardwareExists || shooter.atSpeed().getAsBoolean())
                .andThen(
                    indexer
                        .set(IndexerState.kFeeding)
                        .alongWith(
                            intake.feedShooter(),
                            Commands.sequence(
                                agitateArm(), intake.setPivot(IntakePivotState.kRaised)))));
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
    NamedCommands.registerCommand("Shoot", shoot().withTimeout(3.0).asProxy());
    NamedCommands.registerCommand("DeployIntake", deployIntake().asProxy());
    NamedCommands.registerCommand(
        "StowIntake", intake.setPersistentPivot(IntakePivotState.kStowed));
    NamedCommands.registerCommand("SpinUp", shooter.setPersistent(ShooterState.kShootingNear));

    // Run the intake for a whole region of a path rather than at a single point.
    zoneTrigger("IntakeStart", "IntakeStop").whileTrue(intakeFuel().asProxy());
  }

  /**
   * Scales driver joystick input. Called three times per loop by the default drive command — twice
   * for translation (x and y) and once for rotation — so it must be cheap and side-effect free.
   *
   * @param rotation true when scaling the rotation axis (rad/s), false for translation (m/s)
   * @param slow held to drop to slow mode; otherwise the robot drives at full speed
   */
  public double getDriveMultiplier(boolean rotation, Trigger slow) {
    if (slow.getAsBoolean()) {
      return rotation ? DriveConstants.transferSpeedW() : DriveConstants.TRANSFER_SPEED.get();
    }
    return rotation ? DriveConstants.maxSpeedW() : DriveConstants.MAX_SPEED.get();
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
