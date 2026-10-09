package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import java.util.List;
import java.util.Optional;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.Singular;

/**
 * Hardware wiring for an {@link AngularIOSparkFlex}. The SPARK Flex counterpart of {@link
 * AngularIOTalonFXConfig}.
 *
 * <p>Deliberate differences from the TalonFX config, all forced by the hardware:
 *
 * <ul>
 *   <li><b>No {@code bus}.</b> SPARKs only live on the roboRIO CAN bus, so there is nothing to
 *       choose and no {@code SignalIOManager} batching to join.
 *   <li><b>No {@code sensorId}.</b> A SPARK cannot fuse a remote CTRE CANcoder the way a TalonFX
 *       can. Absolute feedback would have to come from a SPARK-attached encoder instead.
 *   <li><b>Current limiting differs from CTRE supply/stator limits.</b> {@code smartCurrentLimit}
 *       regulates motor phase current by reducing output voltage; {@code secondaryCurrentLimit}
 *       briefly disables output when its threshold is exceeded.
 *   <li><b>{@code inverted} is a boolean</b> rather than a CTRE {@code InvertedValue}.
 * </ul>
 *
 * {@code neutralMode} stays a CTRE {@link NeutralModeValue} purely because {@link AngularIO}'s
 * interface uses it; the IO maps it to REV's {@code IdleMode}. The IO estimates supply-current
 * telemetry from measured motor output current times absolute applied duty cycle.
 */
@Builder
@Getter
public class AngularIOSparkFlexConfig {
  private final int masterId;
  @Singular private final List<Integer> followerIds;

  /** Optional per-motor cached telemetry; enable only for mechanisms being investigated. */
  @Builder.Default private final boolean logFollowerTelemetry = false;

  /** Followers spin opposite the master (gearbox reverses them). */
  @Builder.Default private final boolean opposeMaster = false;

  /**
   * Follower CAN IDs that spin opposite the master, for mechanisms where only some followers are
   * mirrored. A follower listed here is opposed even when {@link #opposeMaster} is false.
   */
  @Singular("opposedFollowerId")
  private final List<Integer> opposedFollowerIds;

  public boolean isFollowerOpposed(int followerId) {
    return opposeMaster || opposedFollowerIds.contains(followerId);
  }

  /**
   * Velocity filter on the motor's built-in encoder. The defaults (32 ms window, depth 8) match the
   * SPARK's own and lag a fast flywheel enough to make velocity P oscillate; shorten them for
   * high-speed mechanisms.
   */
  @Builder.Default private final int encoderMeasurementPeriodMs = 32;

  @Builder.Default private final int encoderAverageDepth = 8;

  @Builder.Default private final Angle resetAngle = Radians.of(0.0);
  @Builder.Default private final Angle softMinAngle = Radians.of(Double.NEGATIVE_INFINITY);
  @Builder.Default private final Angle softMaxAngle = Radians.of(Double.POSITIVE_INFINITY);

  /** Motor rotations per rotation of the mechanism output. */
  @Builder.Default private final double motorRotationsPerOutputRotations = 1.0;

  /** Mechanism travel per output rotation — {@code Rotations.of(1.0)} for a plain pivot. */
  @Builder.Default private final Angle outputAnglePerOutputRotation = Rotation.of(1.0);

  @Builder.Default private final boolean inverted = false;

  /** REV's Smart Current Limit in motor phase amperes. */
  private final Current smartCurrentLimit;

  /**
   * Secondary overcurrent limit in amperes; briefly chops output above the threshold. Leave null to
   * skip configuring it.
   */
  private final Current secondaryCurrentLimit;

  @Builder.Default @Setter private NeutralModeValue neutralMode = NeutralModeValue.Brake;

  @Builder.Default @Setter private double kP = 0.0;
  @Builder.Default @Setter private double kI = 0.0;
  @Builder.Default @Setter private double kD = 0.0;
  @Builder.Default @Setter private double kS = 0.0;
  @Builder.Default @Setter private double kV = 0.0;
  @Builder.Default @Setter private double kA = 0.0;
  @Builder.Default @Setter private double kG = 0.0;

  /**
   * Selects REV firmware gravity feedforward. {@code Arm_Cosine} maps {@code kG} volts to {@code
   * kCos}, with {@code kCosRatio} converting feedback position to mechanism rotations; zero angle
   * must be horizontal. {@code Elevator_Static} maps to constant {@code kG}. Empty disables both
   * gravity terms.
   */
  @Builder.Default @Setter private Optional<GravityTypeValue> gravityType = Optional.empty();

  @Builder.Default @Setter private AngularVelocity cruiseVelocity = RotationsPerSecond.of(0.0);

  @Builder.Default @Setter
  private AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0.0);
}
