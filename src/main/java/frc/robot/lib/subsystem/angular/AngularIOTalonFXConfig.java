package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import java.util.List;
import java.util.Optional;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.Singular;

@Builder
@Getter
public class AngularIOTalonFXConfig {
  private final int masterId;
  @Singular private final List<Integer> followerIds;
  private final CANBus bus;
  @Builder.Default private final boolean opposeMaster = false;
  @Builder.Default private final Angle resetAngle = Radians.of(0.0);
  @Builder.Default private final Angle softMinAngle = Radians.of(Double.NEGATIVE_INFINITY);
  @Builder.Default private final Angle softMaxAngle = Radians.of(Double.POSITIVE_INFINITY);
  @Builder.Default private final double motorRotationsPerOutputRotations = 0.0;
  @Builder.Default private final Angle outputAnglePerOutputRotation = Rotation.of(1.0);
  @Builder.Default private final double rotorRotationsPerSensorRotation = 1.0;
  @Builder.Default private final Optional<Integer> sensorId = Optional.empty();
  @Builder.Default private final Frequency statusFrameUpdateRate = Hertz.of(100.0); // Hz

  @Builder.Default
  private final Angle sensorOffset =
      Radians.of(0.0); // This gets added to the target position, useful when using remote sensor

  private final InvertedValue inverted;
  private final Current supplyCurrentLimit;
  private final Current supplyCurrentLower;
  private final Time supplyCurrentLowerTime;

  private final Current statorCurrentLimit;
  @Builder.Default @Setter private NeutralModeValue neutralMode = NeutralModeValue.Brake;
  @Builder.Default @Setter private double kP = 0.0;
  @Builder.Default @Setter private double kI = 0.0;
  @Builder.Default @Setter private double kD = 0.0;
  @Builder.Default @Setter private double kS = 0.0;
  @Builder.Default @Setter private double kV = 0.0;
  @Builder.Default @Setter private double kA = 0.0;
  @Builder.Default @Setter private double kG = 0.0;
  @Builder.Default @Setter private Optional<GravityTypeValue> gravityType = Optional.empty();
  @Builder.Default @Setter private AngularVelocity cruiseVelocity = RotationsPerSecond.of(0.0);

  @Builder.Default @Setter
  private AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0.0);
}
