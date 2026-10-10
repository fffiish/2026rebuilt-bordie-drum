package frc.robot.constants.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final Angle kPivotStowed = Degrees.of(118.0); // operator-confirmed stowed hard stop

  /** Deployed — the arm's normal resting position once auto has put it down. */
  public static final Angle kPivotDeployed = Degrees.of(0.0); // TODO(bringup)

  /** Raised out of the way while shooting, 90 degrees up from deployed. */
  public static final Angle kPivotRaised = kPivotDeployed.plus(Degrees.of(90.0));

  // ----- agitation: a slow up/down shuffle during a shot, to settle FUEL toward the indexer -----

  /** Top of the agitation stroke. Deliberately small — this is a shake, not a lift. */
  public static final Angle kPivotAgitateHigh =
      kPivotDeployed.plus(Degrees.of(30.0)); // raised from 20 at the driver's request

  /** Bottom of the agitation stroke. */
  public static final Angle kPivotAgitateLow = kPivotDeployed;

  /** How long the arm dwells at each end of a stroke. "Slowly" per the driver's request. */
  public static final Time kAgitateDwell = Seconds.of(0.35); // TODO(bringup)

  /** Number of complete up-down cycles before the arm raises to shoot. */
  public static final int kAgitateCycles = 3; // TODO(bringup)

  // ----- current limits -----

  /** Pivot current limit for everything except deploying — holding, agitating, raising to shoot. */
  public static final Current kPivotCurrentLimit = Amps.of(40);

  /**
   * Pivot current limit while deploying (X, auto start, teleop enable). Breaking the hopper and
   * intake free takes more than ordinary motion. Raised only for the deploy itself, then dropped
   * back to {@link #kPivotCurrentLimit}.
   */
  public static final Current kPivotDeployCurrentLimit = Amps.of(80);

  /** Drop back to the normal limit after this long even if the arm has not reported arriving. */
  public static final Time kPivotDeployTimeout = Seconds.of(2.0); // TODO(bringup)

  /** How close counts as "arrived" when sequencing arm moves. */
  public static final Angle kPivotArrivalTolerance = Degrees.of(5.0); // TODO(bringup)

  public static final AngularIOSparkFlexConfig kPivotSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kPivotMotorId)
          .inverted(false) // TODO(bringup): verify direction
          .motorRotationsPerOutputRotations(60.0) // operator-confirmed reduction
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .smartCurrentLimit(kPivotCurrentLimit)
          .secondaryCurrentLimit(Amps.of(120))
          .softMinAngle(kPivotDeployed)
          .softMaxAngle(kPivotStowed)
          .resetAngle(kPivotStowed) // arm starts stowed against its hard stop
          .gravityType(Optional.of(GravityTypeValue.Arm_Cosine))
          .kP(2.0)
          .kV(0.5)
          .kG(0.35)
          .cruiseVelocity(RotationsPerSecond.of(1.0)) // TODO(bringup)
          .acceleration(RotationsPerSecondPerSecond.of(2.0)) // TODO(bringup)
          .build();

  public static final AngularIOSimConfig kPivotSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(1)) // NEO Vortex, confirmed from CAD
          .numMotors(1)
          .moi(PivotSim.estimateMOI(Inches.of(14.0), Pounds.of(8.0))) // TODO(bringup)
          // Unverified uniform-rod geometry, for simulation only. Zero is horizontal.
          .armLengthSupplier(Optional.of(() -> Inches.of(14.0)))
          .realAngleFromSubsystemAngleZeroSupplier(Optional.of(() -> Rotation2d.kZero))
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
          .kP(2.0)
          .kV(0.5)
          .kG(0.35)
          .cruiseVelocity(RotationsPerSecond.of(1.0))
          .acceleration(RotationsPerSecondPerSecond.of(2.0))
          .build(); // Retain the nonzero gains used in the current robot build.

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

  public static final AngularVelocity kFeederIntaking = RotationsPerSecond.of(15.0);
  public static final AngularVelocity kFeederEjecting =
      RotationsPerSecond.of(-40.0); // TODO(bringup)

  public static final AngularIOSparkFlexConfig kFeederSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kFeederMasterId)
          .followerId(kFeederFollowerId)
          .opposeMaster(true) // Confirmed: CAN 35 must turn opposite CAN 28 to drive together.
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0) // TODO(bringup): real gear ratio
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .smartCurrentLimit(Amps.of(60))
          .secondaryCurrentLimit(Amps.of(60))
          .logFollowerTelemetry(true)
          .kP(0.01 / 12.0)
          .kV(0.12 / (2.0 * Math.PI)) // 0.12 V per rps, expressed as V per rad/s
          .build();

  public static final AngularIOSimConfig kFeederSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(2)) // NEO Vortex, confirmed from CAD
          .numMotors(2)
          .moi(KilogramSquareMeters.of(0.004)) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0)
          .supplyCurrentLimit(Amps.of(60))
          .statorCurrentLimit(Amps.of(60))
          .kV(0.12) // TODO(bringup)
          // Sim-only starting profile: three seconds to the 15 rps forward goal.
          .acceleration(RotationsPerSecondPerSecond.of(5.0))
          .build();

  public static final AngularSubsystemConfig kFeederSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Feeder")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .kP(0.01 / 12.0)
          .kV(0.12 / (2.0 * Math.PI)) // 1.8 V feedforward at the 15 rps target
          .build(); // TODO(bringup): tune kS/kV/kP against the real rollers

  public static final AngularSubsystemConfig kFeederSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey("Feeder")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .kV(0.12)
          .acceleration(RotationsPerSecondPerSecond.of(5.0)) // sim-only, unverified
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
          .smartCurrentLimit(Amps.of(80))
          .secondaryCurrentLimit(Amps.of(120))
          .kV(0.12 / (2.0 * Math.PI)) // 0.12 V per rps, expressed as V per rad/s
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
          .acceleration(RotationsPerSecondPerSecond.of(5.0)) // sim-only, unverified
          .build();

  public static final AngularSubsystemConfig kIntakeRollerSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("IntakeRollers")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .kV(0.12 / (2.0 * Math.PI)) // 6 V feedforward at the 50 rps target
          .build(); // TODO(bringup): tune kS/kV/kP

  public static final AngularSubsystemConfig kIntakeRollerSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey("IntakeRollers")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .kV(0.12)
          .acceleration(RotationsPerSecondPerSecond.of(5.0)) // sim-only, unverified
          .build();
}
