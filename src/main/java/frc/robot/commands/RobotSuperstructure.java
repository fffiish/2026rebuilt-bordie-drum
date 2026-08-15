package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriveConstants;
import frc.robot.lib.command.CachedTrigger;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Cross-subsystem coordination. Individual subsystems own their own state machines; anything that
 * requires two or more of them to agree lives here.
 *
 * <p>TEMPLATE NOTE: reduced to drive-only. This class takes no subsystems today — add them as
 * constructor parameters as you build them, and update the {@code new RobotSuperstructure()} call
 * in {@link frc.robot.RobotContainer}.
 */
public class RobotSuperstructure {

  // TODO(template): take your subsystems as constructor parameters, e.g.
  //   private final Intake intake;
  //   public RobotSuperstructure(Intake intake) { this.intake = intake; }
  public RobotSuperstructure() {}

  /**
   * Registers everything PathPlanner/Choreo autos can reference by name. Called once from {@link
   * frc.robot.RobotContainer}'s constructor, before {@code AutoBuilder.buildAutoChooser()} reads
   * the deploy directory, so every name an {@code .auto} file mentions must be registered by the
   * time this returns.
   *
   * <p>Two mechanisms are available:
   *
   * <ul>
   *   <li>{@code NamedCommands.registerCommand(name, command)} — a command the auto runs as a step.
   *       Wrap it in {@code .asProxy()} when it requires a subsystem the path command does not, so
   *       the scheduler does not cancel the path.
   *   <li>{@link #zoneTrigger(String, String)} — a start/stop event pair that stays true for a
   *       region of the path, for things that should run <em>while</em> driving.
   * </ul>
   */
  public void registerAutoCommands() {
    // TODO(template): register auto commands, e.g.
    //   NamedCommands.registerCommand("Shoot", shooter.shoot().asProxy());
    //   zoneTrigger("IntakeStart", "IntakeStop").whileTrue(intake.set(IntakeState.kIntaking));
  }

  /**
   * Scales driver joystick input. Called three times per loop by the default drive command — twice
   * for translation (x and y) and once for rotation — so it must be cheap and side-effect free.
   *
   * <p>TEMPLATE NOTE: currently just turbo vs. base speed. The season version also slowed the robot
   * while shooting and while intaking; add those terms back here once those subsystems exist, by
   * reading their target state rather than by tracking extra booleans.
   *
   * @param rotation true when scaling the rotation axis (rad/s), false for translation (m/s)
   * @param turbo held to unlock full speed
   */
  public double getDriveMultiplier(boolean rotation, Trigger turbo) {
    if (turbo.getAsBoolean()) {
      return rotation ? DriveConstants.MAX_SPEED_W : DriveConstants.MAX_SPEED.get();
    }
    return rotation ? DriveConstants.TRANSFER_SPEED_W : DriveConstants.TRANSFER_SPEED.get();
  }

  /**
   * A single command that exercises every mechanism through its range of motion, bound to an auto
   * so it can be run from the driver station during pit checks.
   *
   * <p>TEMPLATE NOTE: empty. Add one step per mechanism as you build it — each step should be
   * short, {@code .withTimeout(...)}-bounded, and {@code .asProxy()}-wrapped.
   */
  public Command fullRobotCheck() {
    return sequence().asProxy();
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
