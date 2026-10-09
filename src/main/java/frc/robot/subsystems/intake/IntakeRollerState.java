package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.intake.IntakeConstants;

/**
 * What the rollers on the arm are doing — the floor-pickup rollers (CAN 2 / 37) and the feeder that
 * lifts FUEL into the hopper (CAN 28 / 35). Commanded independently of the pivot.
 */
public enum IntakeRollerState {
  kOff(RotationsPerSecond.of(0.0), RotationsPerSecond.of(0.0)),

  /** Pull FUEL in off the floor. */
  kIntaking(IntakeConstants.kIntakeRollerIntaking, IntakeConstants.kFeederIntaking),

  /** Reverse — spit FUEL back out, or clear a jam. */
  kEjecting(IntakeConstants.kIntakeRollerEjecting, IntakeConstants.kFeederEjecting);

  private final AngularVelocity intakeRollerVelocity;
  private final AngularVelocity feederVelocity;

  IntakeRollerState(AngularVelocity intakeRollerVelocity, AngularVelocity feederVelocity) {
    this.intakeRollerVelocity = intakeRollerVelocity;
    this.feederVelocity = feederVelocity;
  }

  public AngularVelocity getIntakeRollerVelocity() {
    return intakeRollerVelocity;
  }

  public AngularVelocity getFeederVelocity() {
    return feederVelocity;
  }
}
