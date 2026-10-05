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
 *   <li><b>One current limit, not two.</b> {@code smartCurrentLimit} is REV's single supply-side
 *       limit; {@code secondaryCurrentLimit} is the hard shutoff above it.
 *   <li><b>{@code inverted} is a boolean</b> rather than a CTRE {@code InvertedValue}.
 * </ul>
 *
 * {@code neutralMode} stays a CTRE {@link NeutralModeValue} purely because {@link AngularIO}'s
 * interface uses it; the IO maps it to REV's {@code IdleMode}.
 */
@Builder
@Getter
public class AngularIOSparkFlexConfig {
  private final int masterId;
  @Singular private final List<Integer> followerIds;

  /** Followers spin opposite the master (gearbox reverses them). */
  @Builder.Default private final boolean opposeMaster = false;

  @Builder.Default private final Angle resetAngle = Radians.of(0.0);
  @Builder.Default private final Angle softMinAngle = Radians.of(Double.NEGATIVE_INFINITY);
  @Builder.Default private final Angle softMaxAngle = Radians.of(Double.POSITIVE_INFINITY);

  /** Motor rotations per rotation of the mechanism output. */
  @Builder.Default private final double motorRotationsPerOutputRotations = 1.0;

  /** Mechanism travel per output rotation — {@code Rotations.of(1.0)} for a plain pivot. */
  @Builder.Default private final Angle outputAnglePerOutputRotation = Rotation.of(1.0);

  @Builder.Default private final boolean inverted = false;

  /** REV's smart (supply-side) current limit. */
  private final Current smartCurrentLimit;

  /** Hard shutoff limit. Leave null to skip. */
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
   * A SPARK has no firmware gravity term, so the IO applies {@code kG} itself as an arbitrary
   * feedforward — scaled by cos(angle) when this is {@code Arm_Cosine}, constant when {@code
   * Elevator_Static}, and omitted when empty.
   */
  @Builder.Default @Setter private Optional<GravityTypeValue> gravityType = Optional.empty();

  @Builder.Default @Setter private AngularVelocity cruiseVelocity = RotationsPerSecond.of(0.0);

  @Builder.Default @Setter
  private AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0.0);
}
