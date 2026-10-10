package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.subsystem.angular.AngularIOSimConfig;
import frc.robot.lib.subsystem.angular.AngularIOSparkFlexConfig;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;

/**
 * Configs for the shooter: a four-motor flywheel plus a linear-actuator hood.
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
  public static final AngularVelocity kFlywheelMaximumSpeed = RotationsPerSecond.of(40.0);
  public static final AngularVelocity kFlywheelShooting = kFlywheelMaximumSpeed;
  public static final AngularVelocity kFlywheelShootingFar = kFlywheelMaximumSpeed;
  public static final AngularVelocity kFlywheelEjecting =
      RotationsPerSecond.of(20.0); // TODO(bringup)

  // SPARK P uses duty cycle per radian/second; kV uses volts per radian/second.
  // P corresponds to 0.01 V/(rad/s) at the nominal 12 V bus (~0.06 V per rev/s of error). It was
  // 0.08, about 5x kV, which with the SPARK's velocity filter lag made the flywheel oscillate;
  // kV does the bulk of the work and P only trims the residual.
  public static final double kFlywheelKP = 0.01 / 12.0;
  // Measured 10-10: holding 242 rad/s took 4.46 V with no balls -> 0.0184.
  public static final double kFlywheelKV = 0.0184;

  // 39/34/26/29 run open loop at a fixed voltage; the velocity loop oscillated.
  public static final Voltage kFlywheelIdleVoltage = Volts.of(0.0);
  public static final Voltage kFlywheelShootingVoltage = Volts.of(6.7);
  // Lower-power shot: right trigger while holding A.
  public static final Voltage kFlywheelShootingSoftVoltage = Volts.of(5.7);
  public static final Voltage kFlywheelShootingFarVoltage = kFlywheelShootingVoltage;
  public static final Voltage kFlywheelEjectingVoltage = Volts.of(1.25);
  // With no speed setpoint, feeding starts once every motor reaches this fraction of the speed the
  // voltage settles at (volts / kV): 2.5 V settles near 136 rad/s, so feeding starts at 109.
  public static final double kFlywheelReadyFraction = 0.8;

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
          .smartCurrentLimit(Amps.of(40)) // per motor
          .secondaryCurrentLimit(Amps.of(60))
          .maximumMotorVelocity(kFlywheelMaximumSpeed)
          // Four motors starting at once drew ~35-40 A each and pulled the bus to ~6.5 V; ease in.
          .rampRateSeconds(0.4)
          .recoverWhileEnabled(true) // a fault must not kill the drum until the next disable
          .encoderMeasurementPeriodMs(10)
          .encoderAverageDepth(2)
          .logFollowerTelemetry(true)
          .kP(kFlywheelKP)
          .kV(kFlywheelKV)
          .build();

  public static final AngularIOSimConfig kFlywheelSimConfig =
      AngularIOSimConfig.builder()
          .motor(DCMotor.getNeoVortex(4)) // NEO Vortex, confirmed from CAD
          .numMotors(4)
          .moi(KilogramSquareMeters.of(0.012)) // TODO(bringup): real flywheel inertia matters here
          .motorRotationsPerOutputRotations(1.0)
          .supplyCurrentLimit(Amps.of(40))
          .statorCurrentLimit(Amps.of(60))
          .kV(0.11) // TODO(bringup)
          .build();

  public static final AngularSubsystemConfig kFlywheelSubsystemConfigReal =
      AngularSubsystemConfig.builder()
          .logKey("ShooterFlywheel")
          .maximumVelocity(kFlywheelMaximumSpeed)
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(0.5)) // gates feeding near the 40 rps target
          .kP(kFlywheelKP)
          .kV(kFlywheelKV)
          .build();

  public static final AngularSubsystemConfig kFlywheelSubsystemConfigSim =
      AngularSubsystemConfig.builder()
          .logKey("ShooterFlywheel")
          .maximumVelocity(kFlywheelMaximumSpeed)
          .bus(RobotConstants.kRioBus)
          .velocityTolerance(RotationsPerSecond.of(0.5))
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

  // Both actuators are driven from PWM motor controllers on the roboRIO. The RIO's PWM header is
  // signal only, so each channel drives a controller (REV Spark, Victor SP, Talon SR) whose output
  // terminals carry 12 V to the actuator.
  // Three wirings are implemented; RobotContainer picks one.
  //   HoodIOSparkMax (in use) — SPARK MAX on CAN, brushed mode
  //   HoodIOPWM               — a PWM motor controller on the PWM header
  //   HoodIORelay             — Spike or equivalent on the RELAY header
  /** SPARK MAX driving the actuators. Brushed: a PA-14 is a 2-wire DC motor, not a NEO. */
  public static final int kHoodSparkMaxId = 3;

  /** Brushed DC actuators stall at their internal end-stops, so keep this conservative. */
  public static final Current kHoodCurrentLimit = Amps.of(20); // TODO(bringup)

  public static final int kHoodLeftPwmChannel = 0; // only if using HoodIOPWM
  public static final int kHoodRightPwmChannel = 1; // only if using HoodIOPWM
  public static final int kHoodLeftRelayChannel = 0; // only if using HoodIORelay
  public static final int kHoodRightRelayChannel = 1; // only if using HoodIORelay

  /** PA-14P potentiometer wipers, on roboRIO analog inputs. */
  public static final int kHoodLeftAnalogChannel = 0; // TODO(bringup): real channel

  public static final int kHoodRightAnalogChannel = 1; // TODO(bringup): real channel

  /**
   * How far the two sides may disagree, in normalised travel, before the hood is treated as racked.
   * Both actuators drive one surface, so a persistent split means one side is binding or stalled.
   */
  public static final double kHoodSideDisagreement = 0.15; // TODO(bringup)

  /** Potentiometer voltage at each hard stop — calibrate by driving to the ends and reading. */
  public static final double kHoodSensorVoltsRetracted = 0.2; // TODO(bringup): measure

  public static final double kHoodSensorVoltsExtended = 4.8; // TODO(bringup): measure
}
