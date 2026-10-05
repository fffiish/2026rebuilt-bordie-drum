package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOSparkFlexConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import frc.robot.lib.subsystem.linear.LinearIOSimConfig;
import frc.robot.lib.subsystem.linear.LinearIOSparkFlexConfig;
import frc.robot.lib.subsystem.linear.LinearSubsystemConfig;

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

  public static final int kFlywheelMasterId = 21; // TODO(bringup): real CAN id
  public static final int kFlywheelFollowerId = 23; // TODO(bringup): real CAN id

  public static final AngularVelocity kFlywheelIdle = RotationsPerSecond.of(0.0);
  public static final AngularVelocity kFlywheelShooting =
      RotationsPerSecond.of(80.0); // TODO(bringup)
  public static final AngularVelocity kFlywheelEjecting =
      RotationsPerSecond.of(20.0); // TODO(bringup)

  public static final AngularIOSparkFlexConfig kFlywheelSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kFlywheelMasterId)
          .followerId(kFlywheelFollowerId)
          .opposeMaster(false) // TODO(bringup): true if the two wheels face each other
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0) // TODO(bringup): real gear ratio
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .smartCurrentLimit(Amps.of(60))
          .secondaryCurrentLimit(Amps.of(100))
          .build();

  public static final AngularIOSimConfig kFlywheelSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(2)) // NEO Vortex, confirmed from CAD
          .numMotors(2)
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

  public static final int kHoodMotorId = 24; // TODO(bringup): real CAN id

  /** Fully retracted — the hard stop the actuator homes against. */
  public static final Distance kHoodRetracted = Inches.of(0.0); // TODO(bringup)

  /** Fully extended. */
  public static final Distance kHoodExtended = Inches.of(6.0); // TODO(bringup)

  public static final Distance kHoodStowed = kHoodRetracted;
  public static final Distance kHoodShooting = Inches.of(3.0); // TODO(bringup)

  public static final LinearIOSparkFlexConfig kHoodSparkFlexConfig =
      LinearIOSparkFlexConfig.builder()
          .masterId(kHoodMotorId)
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(25.0) // TODO(bringup): real leadscrew reduction
          .outputDistancePerOutputRotation(Inches.of(0.25)) // TODO(bringup): leadscrew pitch
          .smartCurrentLimit(Amps.of(30))
          .secondaryCurrentLimit(Amps.of(60))
          .softMinLength(kHoodRetracted)
          .softMaxLength(kHoodExtended)
          .resetLength(kHoodRetracted)
          .cruiseVelocity(InchesPerSecond.of(4.0)) // TODO(bringup)
          .acceleration(InchesPerSecond.per(Second).of(12.0)) // TODO(bringup)
          .build();

  public static final LinearIOSimConfig kHoodSimConfig =
      LinearIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(1)) // NEO Vortex, confirmed from CAD
          .numMotors(1)
          .carriageMass(Pounds.of(4.0)) // TODO(bringup)
          .motorRotationsPerOutputRotations(25.0)
          .outputDistancePerOutputRotation(Inches.of(0.25))
          .physicalMinLength(kHoodRetracted)
          .physicalMaxLength(kHoodExtended)
          .resetLength(kHoodRetracted)
          .supplyCurrentLimit(Amps.of(30))
          .statorCurrentLimit(Amps.of(60))
          .kP(30.0) // TODO(bringup): sim-only starting gain
          .cruiseVelocity(InchesPerSecond.of(4.0))
          .acceleration(InchesPerSecond.per(Second).of(12.0))
          .build();

  public static final LinearSubsystemConfig kHoodSubsystemConfigReal =
      LinearSubsystemConfig.builder()
          .logKey("ShooterHood")
          .bus(RobotConstants.kRioBus)
          .positionTolerance(Inches.of(0.15))
          .cruiseVelocity(InchesPerSecond.of(4.0))
          .acceleration(InchesPerSecond.per(Second).of(12.0))
          .build(); // TODO(bringup): tune kP/kD/kG

  public static final LinearSubsystemConfig kHoodSubsystemConfigSim =
      LinearSubsystemConfig.builder()
          .logKey("ShooterHood")
          .bus(RobotConstants.kRioBus)
          .positionTolerance(Inches.of(0.15))
          .kP(30.0)
          .cruiseVelocity(InchesPerSecond.of(4.0))
          .acceleration(InchesPerSecond.per(Second).of(12.0))
          .build();
}
