package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.intake.IntakeConstants;

/**
 * Every position the intake can be in. The subsystem decides what each one means; bindings only say
 * which one they want.
 */
public enum IntakeState {
  /** Folded inside the frame perimeter, rollers off. Starting configuration. */
  kStowed(IntakeConstants.kPivotStowed, RotationsPerSecond.of(0.0)),

  /** Arm down, rollers off — for driving up to a ball before committing. */
  kDeployed(IntakeConstants.kPivotDeployed, RotationsPerSecond.of(0.0)),

  /** Arm down, rollers pulling FUEL in. */
  kIntaking(IntakeConstants.kPivotDeployed, IntakeConstants.kRollerIntaking),

  /** Arm down, rollers reversed — spit out a jam or a wrong ball. */
  kEjecting(IntakeConstants.kPivotDeployed, IntakeConstants.kRollerEjecting);

  private final Angle pivotAngle;
  private final AngularVelocity rollerVelocity;

  IntakeState(Angle pivotAngle, AngularVelocity rollerVelocity) {
    this.pivotAngle = pivotAngle;
    this.rollerVelocity = rollerVelocity;
  }

  public Angle getPivotAngle() {
    return pivotAngle;
  }

  public AngularVelocity getRollerVelocity() {
    return rollerVelocity;
  }
}
