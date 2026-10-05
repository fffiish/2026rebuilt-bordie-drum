package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.shooter.ShooterConstants;

/**
 * Every configuration the shooter can be in: a flywheel speed paired with a hood extension.
 *
 * <p>Hood positions are <em>extensions in inches</em>, not launch angles — see {@link
 * ShooterConstants}. Once you have a distance-to-extension interpolating table, add a state that
 * reads from it rather than adding more discrete entries here.
 */
public enum ShooterState {
  /** Everything off, hood stowed. */
  kIdle(ShooterConstants.kFlywheelIdle, ShooterConstants.kHoodStowed),

  /** Flywheel at speed, hood at the shooting position. */
  kShooting(ShooterConstants.kFlywheelShooting, ShooterConstants.kHoodShooting),

  /** Gentle forward — dump FUEL without launching it across the field. */
  kEjecting(ShooterConstants.kFlywheelEjecting, ShooterConstants.kHoodStowed);

  private final AngularVelocity flywheelVelocity;
  private final Distance hoodLength;

  ShooterState(AngularVelocity flywheelVelocity, Distance hoodLength) {
    this.flywheelVelocity = flywheelVelocity;
    this.hoodLength = hoodLength;
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheelVelocity;
  }

  public Distance getHoodLength() {
    return hoodLength;
  }
}
