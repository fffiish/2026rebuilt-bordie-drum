package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.*;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

@Builder
@Getter
public class AngularSubsystemConfig {
  private final String logKey;

  private final CANBus bus;

  @Builder.Default @Setter private Angle positionTolerance = Radians.of(Double.POSITIVE_INFINITY);

  @Builder.Default @Setter
  private AngularVelocity velocityTolerance = RadiansPerSecond.of(Double.POSITIVE_INFINITY);

  @Builder.Default @Setter private double kP = 0.0;
  @Builder.Default @Setter private double kI = 0.0;
  @Builder.Default @Setter private double kD = 0.0;
  @Builder.Default @Setter private double kS = 0.0;
  @Builder.Default @Setter private double kV = 0.0;
  @Builder.Default @Setter private double kA = 0.0;
  @Builder.Default @Setter private double kG = 0.0;
  @Builder.Default @Setter private AngularVelocity cruiseVelocity = RadiansPerSecond.of(0.0);

  @Builder.Default @Setter
  private AngularAcceleration acceleration = RadiansPerSecondPerSecond.of(0.0);
}
