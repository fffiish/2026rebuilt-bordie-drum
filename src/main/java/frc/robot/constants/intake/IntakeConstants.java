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

  /** Raised out of the way while shooting; fixed at 90 degrees, independent of deployed. */
  public static final Angle kPivotRaised = Degrees.of(90.0);

  // ----- agitation: a slow up/down sweep during a shot, to settle FUEL toward the indexer -----

  /** Top of the agitation sweep; the bottom is {@link #kPivotDeployed}. */
  public static final Angle kPivotAgitateHigh = Degrees.of(75.0);

  /**
   * How long one stroke takes, resting to {@link #kPivotAgitateHigh} or back. Moving the goal a
   * little each loop keeps the arm slow and steady instead of jerking, so it needs less current.
   */
  public static final Time kAgitateStrokeTime = Seconds.of(0.55);

  /**
   * Pivot current limit while agitating only; back to {@link #kPivotCurrentLimit} after. Raise this
   * if the arm stalls partway up with FUEL on it.
   */
  public static final Current kPivotAgitateCurrentLimit = Amps.of(75);

  // ----- current limits -----

  /** Pivot current limit for everything except deploying — holding, agitating. */
  public static final Current kPivotCurrentLimit = Amps.of(40);

  /**
   * Pivot current limit while the arm sits at its deployed resting angle with nothing moving it.
   * The resting target is a little past where the arm physically stops, so at the normal limit the
   * motor would push into the stop and stall; this keeps it resting lightly on the stop instead.
   * Applied automatically near deployed, and dropped back to {@link #kPivotCurrentLimit} as soon as
   * anything else moves the arm. Raise this if the arm sags off the stop when resting.
   */
  public static final Current kPivotRestCurrentLimit = Amps.of(10); // TODO(bringup)

  /**
   * Pivot current limit for the first, automatic deploy only (auto start). Breaking the hopper and
   * intake free takes more than ordinary motion. Raised only for that deploy, then dropped back to
   * {@link #kPivotCurrentLimit}; X and everything else use the normal limit.
   */
  public static final Current kPivotDeployCurrentLimit = Amps.of(80);

  /** Pivot current limit while deploying with X; same drop back to normal afterwards. */
  public static final Current kPivotManualDeployCurrentLimit = Amps.of(65);

  /** Drop back to the normal limit after this long even if the arm has not reported arriving. */
  public static final Time kPivotDeployTimeout = Seconds.of(2.0); // TODO(bringup)

  /** Pivot current limit while retracting the arm back to stowed (driver B). */
  public static final Current kPivotRetractCurrentLimit = Amps.of(65);

  /** Drop back to the normal limit after this long even if the retract has not arrived. */
  public static final Time kPivotRetractTimeout = Seconds.of(2.0); // TODO(bringup)

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

  /**
   * The roller floor (named "Feeder" in this code): two motors on the arm that lift FUEL from the
   * intake into the hopper. Not the 30/8 feeder, which is {@code IndexerConstants}.
   */
  public static final int kFeederMasterId = 28; // confirmed: "Feeder #28"

  public static final int kFeederFollowerId = 35; // confirmed: "Feeder #35"

  public static final AngularVelocity kFeederIntaking = RotationsPerSecond.of(35.0);
  public static final AngularVelocity kFeederEjecting =
      RotationsPerSecond.of(-35.0); // TODO(bringup)

  /** Feeder smart current limit per motor, e.g. while feeding a shot. */
  public static final Current kFeederCurrentLimit =
      Amps.of(40); // lowered from 60 A to cut peak draw

  /**
   * TEMPORARY: feeder smart current limit while only intaking (not shooting), so feeder plus floor
   * rollers do not overload the battery. Back to {@link #kFeederCurrentLimit} otherwise.
   */
  public static final Current kFeederIntakingCurrentLimit = Amps.of(20);

  public static final AngularIOSparkFlexConfig kFeederSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kFeederMasterId)
          .followerId(kFeederFollowerId)
          .opposeMaster(true) // Confirmed: CAN 35 must turn opposite CAN 28 to drive together.
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0) // TODO(bringup): real gear ratio
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .smartCurrentLimit(kFeederCurrentLimit)
          .secondaryCurrentLimit(
              Amps.of(70)) // above the smart limit so it regulates before the cutoff
          // 35 hit 150 A and a gate-driver fault 150 ms after starting a shot. Ease in, and let a
          // fault clear itself rather than leaving the floor rollers off until the next disable.
          .rampRateSeconds(0.3)
          .recoverWhileEnabled(true)
          .logFollowerTelemetry(true)
          .dashboardName("Feeder")
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
          // Sim-only starting profile: 7 seconds to the unverified 35 rps goal.
          .acceleration(RotationsPerSecondPerSecond.of(5.0))
          .build();

  public static final AngularSubsystemConfig kFeederSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Feeder")
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .kP(0.01 / 12.0)
          .kV(0.12 / (2.0 * Math.PI)) // 4.2 V feedforward at the 35 rps target
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
      RotationsPerSecond.of(40.0); // TODO(bringup)
  public static final AngularVelocity kIntakeRollerEjecting =
      RotationsPerSecond.of(-40.0); // TODO(bringup)

  /** Intake roller smart current limit per motor (intaking). */
  public static final Current kIntakeRollerCurrentLimit = Amps.of(50);

  /** Intake roller smart current limit while ejecting (outtake): 7 A above the normal limit. */
  public static final Current kIntakeRollerEjectingCurrentLimit =
      kIntakeRollerCurrentLimit.plus(Amps.of(7));

  public static final AngularIOSparkFlexConfig kIntakeRollerSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kIntakeRollerMasterId)
          .followerId(kIntakeRollerFollowerId)
          .opposeMaster(true) // TODO(bringup): left and right face opposite ways
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0) // TODO(bringup): real gear ratio
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          .smartCurrentLimit(kIntakeRollerCurrentLimit)
          .secondaryCurrentLimit(Amps.of(80))
          .rampRateSeconds(0.3)
          .recoverWhileEnabled(true)
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
          .kV(0.12 / (2.0 * Math.PI)) // 4.8 V feedforward at the 40 rps target
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
