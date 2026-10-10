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

  // Bottom shooter rollers: CAN 30 leader, CAN 8 follower. They replace 36/22, which drove the
  // roller directly; 30/8 each drive it through a 5:1 reduction.
  public static final int kMasterId = 30;
  public static final int kFollowerId = 8;

  /** Motor rotations per roller rotation. */
  public static final double kGearRatio = 5.0;

  // Below motor free speed so the loop has headroom and current stays lower on a sagging battery.
  public static final AngularVelocity kMaximumMotorSpeed = RotationsPerSecond.of(65.0);
  // Roller speeds: the SPARK reports velocity through the gear ratio.
  public static final AngularVelocity kMaximumSpeed = RotationsPerSecond.of(65.0 / kGearRatio);
  public static final AngularVelocity kFeeding = RotationsPerSecond.of(45.0 / kGearRatio);
  public static final AngularVelocity kIntaking = kMaximumSpeed;
  public static final AngularVelocity kUnjamming = RPM.of(-6784.0 * 0.20 / kGearRatio);

  // 30/8 run open loop at a fixed voltage; the velocity loop oscillated on this mechanism.
  // Free-speed model: V = kS + 0.0175 V per motor rad/s. On 36/22 (1:1), 2.5 V gave ~20.9 roller
  // rps and 3.0 V ~25.5 roller rps. At 5:1 the motor turns 5x as fast for the same roller speed,
  // so V = 0.2 + 5 * (Vold - 0.2): 2.5 V -> 11.7 V. Feeding would need 14.2 V, more than the
  // battery has, so it runs at the full 12 V (~21.5 roller rps free). Under load the 5x torque
  // holds speed better than 36/22 did, so the loaded gap should be smaller.
  public static final Voltage kFeedingVoltage = Volts.of(12.0);
  public static final Voltage kIntakingVoltage = Volts.of(11.7);
  public static final Voltage kUnjammingVoltage = Volts.of(-11.7);

  // Baselines from the 10-10 shooter captures: 0.0175 V per motor rad/s matches the measured
  // flywheel (same motor), and kS covers belt/roller friction. The gains act on roller rad/s, so
  // kV and kP carry the gear ratio. SPARK P is duty per rad/s: 0.01 / 12 per motor rad/s is about
  // 1 V per 100 rad/s of error, the same proportion as the stable flywheel loop.
  public static final double kRollerKV = 0.0175 * kGearRatio;
  public static final double kRollerKS = 0.20;
  public static final double kRollerKP = 0.01 / 12.0 * kGearRatio;

  public static final AngularIOSparkFlexConfig kSparkFlexConfig =
      AngularIOSparkFlexConfig.builder()
          .masterId(kMasterId)
          .followerId(kFollowerId)
          .opposeMaster(true) // CAN 8 turns opposite CAN 30 to drive together, as 22/36 did.
          .inverted(false) // TODO(bringup)
          .motorRotationsPerOutputRotations(kGearRatio)
          .outputAnglePerOutputRotation(Rotations.of(1.0))
          // Smart limit must sit below the secondary. The secondary is a hard cutoff that briefly
          // kills output; with it at 60 A under an 80 A smart limit, a loaded indexer tripped the
          // cutoff before the smart limit ever regulated, and 36/22 stopped mid-run.
          // 50 A / 70 A: through the 5:1 reduction, 50 A gives the roller about 5x the jam torque
          // 36/22 had at the same current.
          // TODO(bringup): near 12 V a stall wants far more than 50 A; watch for cutoff trips.
          .smartCurrentLimit(Amps.of(50))
          .secondaryCurrentLimit(Amps.of(70))
          .rampRateSeconds(0.3) // output rises over 0.3 s, not in one step
          .recoverWhileEnabled(true) // a fault must not stop the feeders until the next disable
          .maximumMotorVelocity(kMaximumMotorSpeed)
          .logFollowerTelemetry(true)
          .kP(kRollerKP)
          .kV(kRollerKV)
          .kS(kRollerKS)
          .build();

  public static final AngularIOSimConfig kSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(2)) // NEO Vortex, confirmed from CAD
          .numMotors(2)
          .moi(KilogramSquareMeters.of(0.003)) // TODO(bringup)
          .motorRotationsPerOutputRotations(kGearRatio)
          .supplyCurrentLimit(Amps.of(60))
          .statorCurrentLimit(Amps.of(60))
          .kP(0.01 * kGearRatio)
          .kV(kRollerKV)
          .acceleration(RotationsPerSecondPerSecond.of(50 / kGearRatio))
          .build();

  public static final AngularSubsystemConfig kSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("Indexer")
          .maximumVelocity(kMaximumSpeed)
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0 / kGearRatio))
          .kP(kRollerKP)
          .kV(kRollerKV)
          .kS(kRollerKS)
          .build();

  public static final AngularSubsystemConfig kSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey("Indexer")
          .maximumVelocity(kMaximumSpeed)
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(5.0 / kGearRatio))
          .kP(0.01 * kGearRatio)
          .kV(kRollerKV)
          .acceleration(RotationsPerSecondPerSecond.of(50 / kGearRatio))
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
