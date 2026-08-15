package frc.robot.lib.subsystem.linear;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.*;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

@Builder
@Getter
public class LinearSubsystemConfig {
  private final String logKey;

  private final CANBus bus;

  @Builder.Default @Setter private Distance positionTolerance = Meters.of(Double.POSITIVE_INFINITY);

  @Builder.Default @Setter
  private LinearVelocity velocityTolerance = MetersPerSecond.of(Double.POSITIVE_INFINITY);

  @Builder.Default @Setter private double kP = 0.0;
  @Builder.Default @Setter private double kI = 0.0;
  @Builder.Default @Setter private double kD = 0.0;
  @Builder.Default @Setter private double kG = 0.0;
  @Builder.Default @Setter private LinearVelocity cruiseVelocity = MetersPerSecond.of(0.0);

  @Builder.Default @Setter
  private LinearAcceleration acceleration = MetersPerSecondPerSecond.of(0.0);
}
