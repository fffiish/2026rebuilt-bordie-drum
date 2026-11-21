package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOSparkFlexConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;

/**
 * Configs for the shooter: a four-motor flywheel.
 *
 * <p>Every value marked {@code TODO(bringup)} is a placeholder, not a measurement.
 */
public final class ShooterConstants {
  private ShooterConstants() {}

  // -------------------------------------------------------------- flywheel

  // Confirmed: all four run in unison on the one 3" drum, so a single leader with three followers.
  public static final int kFlywheelMasterId = 39; // confirmed: "Shooter #39"
  public static final int kFlywheelFollowerIdA = 34; // confirmed: "Shooter #24: 34"
  public static final int kFlywheelFollowerIdB = 26; // confirmed: "Shooter #26"
  public static final int kFlywheelFollowerIdC = 29; // confirmed: "Shooter #29"

  // User-provided total rotating inertia about the drum axis: 6.5 lb*in^2.
  // Includes the entire rotating system; 1:1 motor/drum gearing is still assumed.
  public static final MomentOfInertia kFlywheelMOI =
      KilogramSquareMeters.of(6.5 * 0.45359237 * 0.0254 * 0.0254);
  // Speed setpoints. Derived from the old 6.4 / 5.7 / 1.25 V presets at the 10-10 measured
  // 0.0184 V per rad/s (no balls), so they are provisional until tuned under load.
  public static final AngularVelocity kFlywheelIdle = RotationsPerSecond.of(0.0);
  public static final AngularVelocity kFlywheelShooting = RotationsPerSecond.of(68.0);
  public static final AngularVelocity kFlywheelShootingSoft = RotationsPerSecond.of(49.303);
  public static final AngularVelocity kFlywheelShootingFar = kFlywheelShooting;
  public static final AngularVelocity kFlywheelEjecting = RotationsPerSecond.of(10.812);
  public static final AngularVelocity kFlywheelMaximumSpeed = kFlywheelShooting;

  // WPILib bang-bang on the roboRIO sends voltage only; REV velocity PID is unused.
  public static final Voltage kFlywheelBangBangVoltage = Volts.of(12.0);
  // Fraction of the target speed every motor must reach before the feeders start. Just under 1.0
  // so bang-bang noise on one motor cannot hold the shot back indefinitely.
  public static final double kFlywheelReadyFraction = 0.97;
  public static final double kFlywheelFeedbackMaxAgeSeconds = 0.1;

  public static final AngularIOSparkFlexConfig kFlywheelSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kFlywheelMasterId)
          .followerId(kFlywheelFollowerIdA)
          .followerId(kFlywheelFollowerIdB)
          .followerId(kFlywheelFollowerIdC)
          // The 26/29 side is mounted mirrored from the 39/34 side, so it must spin opposite.
          .opposedFollowerId(kFlywheelFollowerIdB)
          .opposedFollowerId(kFlywheelFollowerIdC)
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0) // TODO(bringup): real gear ratio
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .smartCurrentLimit(Amps.of(37)) // per motor; lowered 3 A from 40 to ease battery load
          .secondaryCurrentLimit(Amps.of(60))
          .maximumMotorVelocity(kFlywheelMaximumSpeed)
          // Bang-bang needs immediate on/off response and must coast when output is zero.
          .neutralMode(NeutralModeValue.Coast)
          .rampRateSeconds(0.0)
          .recoverWhileEnabled(true) // a fault must not kill the drum until the next disable
          .encoderMeasurementPeriodMs(10)
          .encoderAverageDepth(2)
          .logFollowerTelemetry(true)
          .dashboardName("Flywheel")
          .kP(0.0)
          .kV(0.0)
          .build();

  public static final AngularIOSimConfig kFlywheelSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(4)) // NEO Vortex, confirmed from CAD
          .numMotors(4)
          .moi(kFlywheelMOI)
          .motorRotationsPerOutputRotations(1.0)
          .supplyCurrentLimit(Amps.of(40))
          .statorCurrentLimit(Amps.of(60))
          .neutralMode(NeutralModeValue.Coast)
          .kV(0.0)
          .build();

  public static final AngularSubsystemConfig kFlywheelSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("ShooterFlywheel")
          .maximumVelocity(kFlywheelMaximumSpeed)
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(0.5)) // unused by bang-bang readiness
          .kP(0.0)
          .kV(0.0)
          .build();

  public static final AngularSubsystemConfig kFlywheelSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey("ShooterFlywheel")
          .maximumVelocity(kFlywheelMaximumSpeed)
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(0.5))
          .kV(0.0)
          .build();
}
