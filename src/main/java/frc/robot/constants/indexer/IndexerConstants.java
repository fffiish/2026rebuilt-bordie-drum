package frc.robot.constants.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOSparkFlexConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import frc.robot.lib.subsystem.sensor.canrange.CANRangeIOCANRangeConfig;
import frc.robot.lib.subsystem.sensor.canrange.CANRangeSubsystemConfig;

/**
 * Configs for the indexer: the wheel row that pulls FUEL out of the passive hopper and stages it at
 * the shooter throat.
 *
 * <p>The hopper itself has no actuators, so the indexer is the only thing that can break up a jam —
 * hence both a CANrange at the throat (is a ball staged?) and a current-based jam detector.
 *
 * <p>Every value marked {@code TODO(bringup)} is a placeholder, not a measurement.
 */
public final class IndexerConstants {
  private IndexerConstants() {}

  // --------------------------------------------------------------- rollers

  // Bottom shooter rollers, confirmed by the operator: CAN 36 leader, CAN 22 follower.
  public static final int kMasterId = 36; // confirmed: "Shooter #36"
  public static final int kFollowerId = 22; // confirmed: "Shooter #22"

  public static final AngularVelocity kMaximumSpeed = RotationsPerSecond.of(65.0);
  public static final AngularVelocity kFeeding = kMaximumSpeed;
  public static final AngularVelocity kIntaking = kMaximumSpeed;
  public static final AngularVelocity kUnjamming = RPM.of(-6784.0 * 0.20);

  public static final double kRollerKV = 12.0 / RPM.of(6784.0).in(RadiansPerSecond);
  public static final double kRollerKP = 0.01 / 12.0;

  public static final AngularIOSparkFlexConfig kSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kMasterId)
          .followerId(kFollowerId)
          .opposeMaster(true) // Confirmed: CAN 22 must turn opposite CAN 36 to drive together.
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0) // TODO(bringup): real gear ratio
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          // Smart limit must sit below the secondary. The secondary is a hard cutoff that briefly
          // kills output; with it at 60 A under an 80 A smart limit, a loaded indexer tripped the
          // cutoff before the smart limit ever regulated, and 36/22 stopped mid-run.
          .smartCurrentLimit(Amps.of(60))
          .secondaryCurrentLimit(Amps.of(80))
          .rampRateSeconds(0.3) // output rises over 0.3 s, not in one step
          .recoverWhileEnabled(true) // a fault must not stop the feeders until the next disable
          .maximumMotorVelocity(kMaximumSpeed)
          .logFollowerTelemetry(true)
          .kP(kRollerKP)
          .kV(kRollerKV)
          .build();

  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(2)) // NEO Vortex, confirmed from CAD
          .numMotors(2)
          .moi(KilogramSquareMeters.of(0.003)) // TODO(bringup)
          .motorRotationsPerOutputRotations(1.0)
          .supplyCurrentLimit(Amps.of(60))
          .statorCurrentLimit(Amps.of(60))
          .kP(0.01)
          .kV(kRollerKV)
          .acceleration(RotationsPerSecondPerSecond.of(50))
          .build();

  public static final AngularSubsystemConfig kSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Indexer")
          .maximumVelocity(kMaximumSpeed)
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .kP(kRollerKP)
          .kV(kRollerKV)
          .build();

  public static final AngularSubsystemConfig kSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey("Indexer")
          .maximumVelocity(kMaximumSpeed)
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0))
          .kP(0.01)
          .kV(kRollerKV)
          .acceleration(RotationsPerSecondPerSecond.of(50))
          .build();

  // ------------------------------------------------------- staging sensor

  public static final int kCANRangeId = 20; // TODO(bringup): real CAN id

  public static final CANRangeIOCANRangeConfig kCANRangeIOConfig =
      CANRangeIOCANRangeConfig.builder().id(kCANRangeId).bus(RobotConstants.kRioBus).build();

  /**
   * FUEL is a 5.91 in ball, so the free-air reading across the throat and the ball-present reading
   * are far apart. Set this between them with the sensor's live distance on the dashboard.
   */
  public static final CANRangeSubsystemConfig kCANRangeSubsystemConfig =
      CANRangeSubsystemConfig.builder()
          .logKey("IndexerStaged")
          .threshold(Inches.of(4.0)) // TODO(bringup): measure empty vs. ball-present
          .debounce(Seconds.of(0.04))
          .build();

  // ---------------------------------------------------------- jam detector

  /** Supply current above this for the debounce window means the wheel row is stalled. */
  public static final Current kJamCurrentThreshold = Amps.of(35.0); // TODO(bringup)

  public static final Time kJamDebounce = Seconds.of(0.25); // TODO(bringup)
}
