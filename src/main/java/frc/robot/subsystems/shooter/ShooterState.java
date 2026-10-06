package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
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
  kIdle(ShooterConstants.kFlywheelIdle, HoodState.kNear),

  /** Flywheel at speed, hood at the shooting position. */
  kShootingNear(ShooterConstants.kFlywheelShooting, HoodState.kNear),

  /** Same flywheel speed, hood pulled forward — the long-range preset. */
  kShootingFar(ShooterConstants.kFlywheelShootingFar, HoodState.kFar),

  /** Gentle forward — dump FUEL without launching it across the field. */
  kEjecting(ShooterConstants.kFlywheelEjecting, HoodState.kNear);

  private final AngularVelocity flywheelVelocity;
  private final HoodState hoodState;

  ShooterState(AngularVelocity flywheelVelocity, HoodState hoodState) {
    this.flywheelVelocity = flywheelVelocity;
    this.hoodState = hoodState;
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheelVelocity;
  }

  public HoodState getHoodState() {
    return hoodState;
  }
}
