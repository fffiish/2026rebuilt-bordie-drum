package frc.robot.lib.subsystem.linear;

/**
 * Output mode for the Linear Subsystem. Separate from {@link LinearIOOutputMode Linear IO Output
 * Mode}. *
 */
public enum LinearSubsystemOutputMode {
  kClosedLoop,
  kOpenLoop,
  kHoldAtCall,
  kHoldAtGoal
}
