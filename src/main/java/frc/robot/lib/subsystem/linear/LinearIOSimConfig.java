package frc.robot.lib.subsystem.linear;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

@Builder
@Getter
public class LinearIOSimConfig {
  private final DCMotor motor;
  @Builder.Default private final int numMotors = 1;
  @Builder.Default private final Mass carriageMass = Kilograms.of(0.0);
  @Builder.Default private final Distance resetLength = Meters.of(0.0);
  @Builder.Default private final Distance physicalMinLength = Meters.of(Double.NEGATIVE_INFINITY);
  @Builder.Default private final Distance physicalMaxLength = Meters.of(Double.POSITIVE_INFINITY);
  @Builder.Default private final double motorRotationsPerOutputRotations = 1.0;
  @Builder.Default private final Distance outputDistancePerOutputRotation = Meters.of(1.0);
  @Builder.Default @Setter private NeutralModeValue neutralMode = NeutralModeValue.Brake;
  @Builder.Default private final Current supplyCurrentLimit = Amps.of(Double.POSITIVE_INFINITY);
  @Builder.Default private final Current statorCurrentLimit = Amps.of(Double.POSITIVE_INFINITY);

  @Builder.Default @Setter private double kP = 0.0;
  @Builder.Default @Setter private double kI = 0.0;
  @Builder.Default @Setter private double kD = 0.0;
  @Builder.Default @Setter private double kG = 0.0;
  @Builder.Default @Setter private LinearVelocity cruiseVelocity = MetersPerSecond.of(0.0);

  @Builder.Default @Setter
  private LinearAcceleration acceleration = MetersPerSecondPerSecond.of(0.0);
}
