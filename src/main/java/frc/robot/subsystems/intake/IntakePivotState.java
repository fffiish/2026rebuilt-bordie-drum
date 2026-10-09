package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.intake.IntakeConstants;

/**
 * Where the intake arm is. Commanded independently of the rollers, so the driver can intake without
 * moving the arm and a shot can raise the arm without touching the rollers.
 */
public enum IntakePivotState {
  /** Folded inside the frame — the legal starting configuration, before auto deploys it. */
  kStowed(IntakeConstants.kPivotStowed),

  /** Down, the arm's resting position for the rest of the match once deployed. */
  kDeployed(IntakeConstants.kPivotDeployed),

  /** Raised 90 degrees from deployed and held there while shooting. */
  kRaised(IntakeConstants.kPivotRaised),

  /** Top of the pre-shot agitation stroke. */
  kAgitateHigh(IntakeConstants.kPivotAgitateHigh),

  /** Bottom of the pre-shot agitation stroke. */
  kAgitateLow(IntakeConstants.kPivotAgitateLow);

  private final Angle angle;

  IntakePivotState(Angle angle) {
    this.angle = angle;
  }

  public Angle getAngle() {
    return angle;
  }
}
