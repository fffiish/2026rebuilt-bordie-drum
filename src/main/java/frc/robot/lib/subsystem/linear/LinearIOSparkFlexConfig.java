package frc.robot.lib.subsystem.linear;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import java.util.List;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.Singular;

/**
 * Hardware wiring for a {@link LinearIOSparkFlex}. The SPARK Flex counterpart of {@link
 * LinearIOTalonFXConfig}.
 *
 * <p>Same hardware-forced differences as the angular config: no CAN bus selection, one smart
 * current limit instead of separate supply/stator limits, and a boolean {@code inverted}.
 */
@Builder
@Getter
public class LinearIOSparkFlexConfig {
  private final int masterId;
  @Singular private final List<Integer> followerIds;

  @Builder.Default private final boolean opposeMaster = false;

  @Builder.Default private final Distance resetLength = Meters.of(0.0);
  @Builder.Default private final Distance softMinLength = Meters.of(Double.NEGATIVE_INFINITY);
  @Builder.Default private final Distance softMaxLength = Meters.of(Double.POSITIVE_INFINITY);

  @Builder.Default private final double motorRotationsPerOutputRotations = 1.0;
  @Builder.Default private final Distance outputDistancePerOutputRotation = Meters.of(1.0);

  @Builder.Default private final boolean inverted = false;

  private final Current smartCurrentLimit;
  private final Current secondaryCurrentLimit;

  @Builder.Default @Setter private NeutralModeValue neutralMode = NeutralModeValue.Brake;

  @Builder.Default @Setter private double kP = 0.0;
  @Builder.Default @Setter private double kI = 0.0;
  @Builder.Default @Setter private double kD = 0.0;

  /** Applied here as a constant arbitrary feedforward; a SPARK has no firmware gravity term. */
  @Builder.Default @Setter private double kG = 0.0;

  @Builder.Default @Setter private LinearVelocity cruiseVelocity = MetersPerSecond.of(0.0);

  @Builder.Default @Setter
  private LinearAcceleration acceleration = MetersPerSecondPerSecond.of(0.0);
}
