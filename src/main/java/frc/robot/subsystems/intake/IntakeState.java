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
  kStowed(IntakeConstants.kPivotStowed, RotationsPerSecond.of(0.0), RotationsPerSecond.of(0.0)),

  /** Arm down, rollers off — for driving up to a ball before committing. */
  kDeployed(IntakeConstants.kPivotDeployed, RotationsPerSecond.of(0.0), RotationsPerSecond.of(0.0)),

  /** Arm down, rollers pulling FUEL in. */
  kIntaking(
      IntakeConstants.kPivotDeployed,
      IntakeConstants.kIntakeRollerIntaking,
      IntakeConstants.kFeederIntaking),

  /** Arm down, rollers reversed — spit out a jam or a wrong ball. */
  kEjecting(
      IntakeConstants.kPivotDeployed,
      IntakeConstants.kIntakeRollerEjecting,
      IntakeConstants.kFeederEjecting);

  private final Angle pivotAngle;
  private final AngularVelocity intakeRollerVelocity;
  private final AngularVelocity feederVelocity;

  IntakeState(
      Angle pivotAngle, AngularVelocity intakeRollerVelocity, AngularVelocity feederVelocity) {
    this.pivotAngle = pivotAngle;
    this.intakeRollerVelocity = intakeRollerVelocity;
    this.feederVelocity = feederVelocity;
  }

  public Angle getPivotAngle() {
    return pivotAngle;
  }

  /** The floor-pickup rollers on the Intake assembly (CAN 2 / 37). */
  public AngularVelocity getIntakeRollerVelocity() {
    return intakeRollerVelocity;
  }

  /** The Feeder assembly on the arm that lifts FUEL into the hopper (CAN 28 / 35). */
  public AngularVelocity getFeederVelocity() {
    return feederVelocity;
  }
}
