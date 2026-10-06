package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOSparkFlexConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;

/**
 * Configs for the shooter: a two-motor flywheel plus a linear-actuator hood.
 *
 * <p>The hood is tracked in <em>extension</em> (inches), never in launch angle — the crank geometry
 * between actuator travel and hood angle is nonlinear, so the mapping lives in a {@link
 * frc.robot.lib.LoggedInterpolatingTable} from target distance straight to extension. Note that
 * {@link frc.robot.lib.subsystem.linear.LinearSubsystem} publishes its tunables in inches while
 * every angular tunable publishes in radians; that asymmetry is a known library quirk.
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

  public static final AngularVelocity kFlywheelIdle = RotationsPerSecond.of(0.0);
  public static final AngularVelocity kFlywheelShooting =
      RotationsPerSecond.of(80.0); // TODO(bringup): speed for the near hood preset
  public static final AngularVelocity kFlywheelShootingFar =
      RotationsPerSecond.of(95.0); // TODO(bringup): speed for the far hood preset
  public static final AngularVelocity kFlywheelEjecting =
      RotationsPerSecond.of(20.0); // TODO(bringup)

  public static final AngularIOSparkFlexConfig kFlywheelSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kFlywheelMasterId)
          .followerId(kFlywheelFollowerIdA)
          .followerId(kFlywheelFollowerIdB)
          .followerId(kFlywheelFollowerIdC)
          .opposeMaster(false) // TODO(bringup): true if the two wheels face each other
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0) // TODO(bringup): real gear ratio
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .smartCurrentLimit(Amps.of(60))
          .secondaryCurrentLimit(Amps.of(100))
          .build();

  public static final AngularIOSimConfig kFlywheelSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(4)) // NEO Vortex, confirmed from CAD
          .numMotors(4)
          .moi(KilogramSquareMeters.of(0.012)) // TODO(bringup): real flywheel inertia matters here
          .motorRotationsPerOutputRotations(1.0)
          .supplyCurrentLimit(Amps.of(60))
          .statorCurrentLimit(Amps.of(100))
          .kV(0.11) // TODO(bringup)
          .build();

  public static final AngularSubsystemConfig kFlywheelSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("ShooterFlywheel")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(2.0)) // gates "ready to fire"
          .build(); // TODO(bringup): tune kS/kV/kP

  public static final AngularSubsystemConfig kFlywheelSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey("ShooterFlywheel")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(2.0))
          .kV(0.11)
          .build();

  // ------------------------------------------------------------------ hood

  /**
   * Worst-case time for the pair of 12 V actuators to travel between the two hood positions. {@link
   * frc.robot.subsystems.shooter.Hood} has no position feedback, so this timer is the only thing
   * standing between "commanded" and "actually there" — measure it on the real mechanism and round
   * up. Too short and the first shot of a burst leaves before the hood has settled.
   */
  public static final Time kHoodTravelTime = Seconds.of(0.5); // TODO(bringup): measure

  /**
   * Voltage applied to drive the actuators. They stop themselves at the built-in limit switches.
   */
  public static final Voltage kHoodDriveVoltage = Volts.of(12.0);

  /** How close to an end point counts as arrived, in normalised travel (0.0-1.0). */
  public static final double kHoodPositionTolerance = 0.05; // TODO(bringup)

  /** Potentiometer voltage at each hard stop — calibrate by driving to the ends and reading. */
  public static final double kHoodSensorVoltsRetracted = 0.2; // TODO(bringup): measure

  public static final double kHoodSensorVoltsExtended = 4.8; // TODO(bringup): measure
}
