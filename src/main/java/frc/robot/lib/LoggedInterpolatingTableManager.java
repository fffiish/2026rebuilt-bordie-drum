package frc.robot.lib;

import frc.robot.lib.subsystem.VirtualSubsystem;

/**
 * Virtual subsystem that automatically updates all LoggedInterpolatingTable instances. Instantiate
 * this once in RobotContainer to enable automatic table updates.
 */
public class LoggedInterpolatingTableManager extends VirtualSubsystem {
  private static LoggedInterpolatingTableManager instance;

  public LoggedInterpolatingTableManager() {
    if (instance != null) {
      throw new IllegalStateException(
          "LoggedInterpolatingTableManager already exists! Only create one instance.");
    }
    instance = this;
  }

  @Override
  public void periodic() {
    // Update all registered tables
    LoggedInterpolatingTable.periodicAll();
  }

  public static LoggedInterpolatingTableManager getInstance() {
    return instance;
  }
}
