package frc.robot.lib.subsystem.angular;

/**
 * Output mode for the Angular Subsystem. Separate from {@link AngularIOOutputMode Angular IO Output
 * Mode}. *
 */
public enum AngularSubsystemOutputMode {
  kClosedLoop,
  kOpenLoop,
  kHoldAtCall,
  kHoldAtGoal,
  kVelocity
}
