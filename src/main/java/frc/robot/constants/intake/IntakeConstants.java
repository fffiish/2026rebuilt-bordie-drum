package frc.robot.constants.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.sim.PivotSim;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOSparkFlexConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import java.util.Optional;

/**
 * Configs for the intake: a gravity-loaded pivot arm carrying a pair of feeder rollers.
 *
 * <p>Every value marked {@code TODO(bringup)} is a placeholder chosen to be plausible, not
 * measured. The structure is correct; the numbers are not. Grep {@code TODO(bringup)} for the full
 * list. Gains of zero mean "not yet tuned" — the mechanism will not hold position until they are.
 */
public final class IntakeConstants {
  private IntakeConstants() {}

  // ---------------------------------------------------------------- pivot

  public static final int kPivotMotorId = 21; // confirmed: "Intake pivot"

  /** Stowed, inside the frame perimeter. */
  public static final Angle kPivotStowed = Degrees.of(95.0); // TODO(bringup)

  /** Deployed, rollers on the floor. */
  public static final Angle kPivotDeployed = Degrees.of(0.0); // TODO(bringup)

  public static final AngularIOSparkFlexConfig kPivotSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kPivotMotorId)
          .inverted(false) // TODO(bringup): verify direction
          .motorRotationsPerOutputRotations(60.0) // TODO(bringup): real gear ratio
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .smartCurrentLimit(Amps.of(40))
          .secondaryCurrentLimit(Amps.of(80))
          .softMinAngle(kPivotDeployed)
          .softMaxAngle(kPivotStowed)
          .resetAngle(kPivotStowed) // arm starts stowed against its hard stop
          .gravityType(Optional.of(GravityTypeValue.Arm_Cosine))
          .cruiseVelocity(RotationsPerSecond.of(1.0)) // TODO(bringup)
          .acceleration(RotationsPerSecondPerSecond.of(2.0)) // TODO(bringup)
          .build();

  public static final AngularIOSimConfig kPivotSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(1)) // NEO Vortex, confirmed from CAD
          .numMotors(1)
          .moi(PivotSim.estimateMOI(Inches.of(14.0), Pounds.of(8.0))) // TODO(bringup)
          .motorRotationsPerOutputRotations(60.0)
          .physicalMinAngle(kPivotDeployed)
          .physicalMaxAngle(kPivotStowed)
          .resetAngle(kPivotStowed)
          .kgArm(true)
          .supplyCurrentLimit(Amps.of(40))
          .statorCurrentLimit(Amps.of(80))
          .kP(8.0) // TODO(bringup): sim-only starting gain
          .kV(0.5)
          .kG(0.35)
          .cruiseVelocity(RotationsPerSecond.of(1.0))
          .acceleration(RotationsPerSecondPerSecond.of(2.0))
          .build();

  public static final AngularSubsystemConfig kPivotSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("IntakePivot")
          .bus(RobotConstants.kRioBus)
          .positionTolerance(Degrees.of(2.0))
          .velocityTolerance(DegreesPerSecond.of(10.0))
          .cruiseVelocity(RotationsPerSecond.of(1.0))
          .acceleration(RotationsPerSecondPerSecond.of(2.0))
          .build(); // TODO(bringup): kP/kD/kS/kV/kG all zero until tuned on the real arm

  public static final AngularSubsystemConfig kPivotSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey("IntakePivot")
          .bus(RobotConstants.kRioBus)
          .positionTolerance(Degrees.of(2.0))
          .velocityTolerance(DegreesPerSecond.of(10.0))
          .kP(8.0)
          .kV(0.5)
          .kG(0.35)
          .cruiseVelocity(RotationsPerSecond.of(1.0))
          .acceleration(RotationsPerSecondPerSecond.of(2.0))
          .build();

  // ------------------------------------------------- feeder rollers (on the arm)

  /** The Feeder assembly: two motors on the arm that lift FUEL from the intake into the hopper. */
  public static final int kFeederMasterId = 28; // confirmed: "Feeder #28"

  public static final int kFeederFollowerId = 35; // confirmed: "Feeder #35"

  public static final AngularVelocity kFeederIntaking =
      RotationsPerSecond.of(50.0); // TODO(bringup)
  public static final AngularVelocity kFeederEjecting =
      RotationsPerSecond.of(-40.0); // TODO(bringup)

  public static final AngularIOSparkFlexConfig kFeederSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kFeederMasterId)
          .followerId(kFeederFollowerId)
          .opposeMaster(false) // TODO(bringup): true if the second motor faces the other way
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0) // TODO(bringup): real gear ratio
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .smartCurrentLimit(Amps.of(40))
          .secondaryCurrentLimit(Amps.of(80))
          .build();

  public static final AngularIOSimConfig kFeederSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(2)) // NEO Vortex, confirmed from CAD
          .numMotors(2)
          .moi(KilogramSquareMeters.of(0.004)) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0)
          .supplyCurrentLimit(Amps.of(40))
          .statorCurrentLimit(Amps.of(80))
          .kV(0.12) // TODO(bringup)
          .build();

  public static final AngularSubsystemConfig kFeederSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Feeder")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .build(); // TODO(bringup): tune kS/kV/kP against the real rollers

  public static final AngularSubsystemConfig kFeederSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey("Feeder")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .kV(0.12)
          .build();

  // ---------------------------------------------------- intake rollers (floor pickup)

  /** The Intake assembly's own roller pair, left and right, driven in unison. */
  public static final int kIntakeRollerMasterId = 2; // confirmed: "Intake left"

  public static final int kIntakeRollerFollowerId = 37; // confirmed: "Intake right"

  public static final AngularVelocity kIntakeRollerIntaking =
      RotationsPerSecond.of(50.0); // TODO(bringup)
  public static final AngularVelocity kIntakeRollerEjecting =
      RotationsPerSecond.of(-40.0); // TODO(bringup)

  public static final AngularIOSparkFlexConfig kIntakeRollerSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kIntakeRollerMasterId)
          .followerId(kIntakeRollerFollowerId)
          .opposeMaster(true) // TODO(bringup): left and right face opposite ways
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0) // TODO(bringup): real gear ratio
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .smartCurrentLimit(Amps.of(40))
          .secondaryCurrentLimit(Amps.of(80))
          .build();

  public static final AngularIOSimConfig kIntakeRollerSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(2)) // NEO Vortex, confirmed from CAD
          .numMotors(2)
          .moi(KilogramSquareMeters.of(0.004)) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0)
          .supplyCurrentLimit(Amps.of(40))
          .statorCurrentLimit(Amps.of(80))
          .kV(0.12) // TODO(bringup)
          .build();

  public static final AngularSubsystemConfig kIntakeRollerSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("IntakeRollers")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .build(); // TODO(bringup): tune kS/kV/kP

  public static final AngularSubsystemConfig kIntakeRollerSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey("IntakeRollers")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .kV(0.12)
          .build();
}
