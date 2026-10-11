package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.shooter.ShooterConstants;

/**
 * Every configuration the shooter can be in: an open-loop flywheel voltage paired with a hood
 * extension.
 *
 * <p>Hood positions are <em>extensions in inches</em>, not launch angles — see {@link
 * ShooterConstants}. Once you have a distance-to-extension interpolating table, add a state that
 * reads from it rather than adding more discrete entries here.
 */
public enum ShooterState {
  /** Everything off, hood stowed. */
  kIdle(ShooterConstants.kFlywheelIdleVoltage, HoodState.kNear),

  /** Flywheel at speed, hood at the shooting position. */
  kShootingNear(ShooterConstants.kFlywheelShootingVoltage, HoodState.kNear),

  /** Lower flywheel voltage, hood at the shooting position — right trigger while holding A. */
  kShootingSoft(ShooterConstants.kFlywheelShootingSoftVoltage, HoodState.kNear),

  /** Same flywheel speed, hood pulled forward — the long-range preset. */
  kShootingFar(ShooterConstants.kFlywheelShootingFarVoltage, HoodState.kFar),

  /** Gentle forward — dump FUEL without launching it across the field. */
  kEjecting(ShooterConstants.kFlywheelEjectingVoltage, HoodState.kNear);

  private final Voltage flywheelVoltage;
  private final HoodState hoodState;

  ShooterState(Voltage flywheelVoltage, HoodState hoodState) {
    this.flywheelVoltage = flywheelVoltage;
    this.hoodState = hoodState;
  }

  public Voltage getFlywheelVoltage() {
    return flywheelVoltage;
  }

  public HoodState getHoodState() {
    return hoodState;
  }
}
