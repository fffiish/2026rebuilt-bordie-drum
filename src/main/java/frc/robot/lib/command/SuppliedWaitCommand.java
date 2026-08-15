package frc.robot.lib.command;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/**
 * A command that does nothing but takes a specified amount of time to finish. Useful for
 * CommandGroups. Can also be subclassed to make a command with an internal timer.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("unused")
public class SuppliedWaitCommand extends Command {
  protected Timer m_timer = new Timer();
  private final DoubleSupplier m_duration;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds A supplier for the time to wait, in seconds
   */
  public SuppliedWaitCommand(DoubleSupplier seconds) {
    m_duration = seconds;
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration.getAsDouble());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("duration", () -> m_duration.getAsDouble(), null);
  }
}
