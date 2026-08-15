package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

@Builder
@Getter
public class AngularIOSimConfig {
  private final DCMotor motor;
  @Builder.Default private final int numMotors = 1;
  @Builder.Default private final MomentOfInertia moi = KilogramSquareMeters.of(1.0);
  @Builder.Default private final Angle resetAngle = Radians.of(0.0);
  @Builder.Default private final Angle physicalMinAngle = Radians.of(Double.NEGATIVE_INFINITY);
  @Builder.Default private final Angle physicalMaxAngle = Radians.of(Double.POSITIVE_INFINITY);
  @Builder.Default private final double motorRotationsPerOutputRotations = 1.0;
  @Builder.Default @Setter private NeutralModeValue neutralMode = NeutralModeValue.Brake;
  @Builder.Default private final Current supplyCurrentLimit = Amps.of(Double.POSITIVE_INFINITY);
  @Builder.Default private final Current statorCurrentLimit = Amps.of(Double.POSITIVE_INFINITY);

  @Builder.Default @Setter private double kP = 0.0;
  @Builder.Default @Setter private double kI = 0.0;
  @Builder.Default @Setter private double kD = 0.0;
  @Builder.Default @Setter private double kV = 0.0;
  @Builder.Default @Setter private double kG = 0.0;
  @Builder.Default @Setter private double kS = 0.0;
  @Builder.Default @Setter private boolean kgArm = false;

  @Builder.Default
  private final Optional<Supplier<Rotation2d>> realAngleFromSubsystemAngleZeroSupplier =
      Optional.empty();

  @Builder.Default private final Optional<Supplier<Distance>> armLengthSupplier = Optional.empty();

  @Builder.Default @Setter private AngularVelocity cruiseVelocity = RotationsPerSecond.of(0.0);

  @Builder.Default @Setter
  private AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0.0);
}
