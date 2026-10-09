package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriveConstants;
import frc.robot.lib.command.CachedTrigger;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;
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
   * Deploy the arm and run the intake rollers so FUEL is pulled off the floor. The indexer is left
   * alone; it only runs from {@link #shoot()}. Runs until cancelled.
   */
  public Command intakeFuel() {
    return intake.set(IntakeState.kIntaking);
  }

  /**
   * Driver trigger: deploy and intake while held; release stows the pivot and stops the rollers.
   */
  public void bindIntakeTrigger(Trigger intakeTrigger) {
    intakeTrigger.whileTrue(intakeFuel());
  }

  /** Driver pickup rollers while held; pivot position is retained on press and release. */
  public Command intakeFuelWithoutDeploy() {
    return intake.runRollers();
  }

  /**
   * Spin the flywheel up and extend the hood, then start feeding once <em>both</em> are in
   * tolerance. Holding the feed off until {@link Shooter#readyToFire()} is what stops the first
   * ball of a burst from going short.
   */
  public Command shoot() {
    return shooter
        .set(ShooterState.kShootingNear)
        .alongWith(
            Commands.waitUntil(shooter.readyToFire()).andThen(indexer.set(IndexerState.kFeeding)));
  }

  /**
   * Reverse the intake and the indexer — spits FUEL back out, and doubles as jam clearing since the
   * hopper is passive and the indexer is the only thing that can break up a pile.
   */
  public Command outtake() {
    return indexer.set(IndexerState.kUnjamming).alongWith(intake.set(IntakeState.kEjecting));
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
    NamedCommands.registerCommand("StowIntake", intake.setPersistent(IntakeState.kStowed));
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
            intake.set(IntakeState.kDeployed).withTimeout(1.5),
            intake.set(IntakeState.kIntaking).withTimeout(1.0),
            intake.set(IntakeState.kStowed).withTimeout(1.5),
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
