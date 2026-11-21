package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.shooter.ShooterConstants;

/** Every configuration the shooter can be in: a flywheel velocity target. */
public enum ShooterState {
  /** Everything off. */
  kIdle(ShooterConstants.kFlywheelIdle),

  /** Flywheel at speed. */
  kShootingNear(ShooterConstants.kFlywheelShooting),

  /** Lower flywheel speed — right trigger while holding A. */
  kShootingSoft(ShooterConstants.kFlywheelShootingSoft),

  /** Same flywheel speed as {@link #kShootingNear}. */
  kShootingFar(ShooterConstants.kFlywheelShootingFar),

  /** Gentle forward — dump FUEL without launching it across the field. */
  kEjecting(ShooterConstants.kFlywheelEjecting);

  private final AngularVelocity flywheelVelocity;

  ShooterState(AngularVelocity flywheelVelocity) {
    this.flywheelVelocity = flywheelVelocity;
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheelVelocity;
  }
}
