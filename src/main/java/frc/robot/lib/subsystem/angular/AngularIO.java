package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amps;
import static frc.robot.lib.subsystem.angular.AngularIOOutputMode.kNeutral;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import org.littletonrobotics.junction.AutoLog;

public interface AngularIO {
  default void updateInputs(AngularIOInputs inputs) {}

  @AutoLog
  class AngularIOInputs {
    // Separate from AngularSubsystemOutputMode.
    public AngularIOOutputMode IOOutputMode = kNeutral;

    public Angle angle = Radians.of(0.0);

    public Voltage appliedVolts = Volts.of(0.0);
    public Current supplyCurrent = Amps.of(0.0);
    public Current statorCurrent = Amps.of(0.0);
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);
    public AngularAcceleration acceleration = RadiansPerSecondPerSecond.of(0.0);

    public NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public double[] motorTemperatures = {};
    public DeviceConnectedStatus[] deviceConnectedStatuses = {};

    public Angle goalPos = Radians.of(0.0);
    public AngularVelocity goalVel = RadiansPerSecond.of(0.0);
    public Angle referencePos = Radians.of(0.0);
    public AngularVelocity referenceVel = RadiansPerSecond.of(0.0);
  }

  default void setAngle(Angle angle) {}

  default void setAngle(Angle angle, Voltage feedforward) {}

  default void setOpenLoop(Voltage voltage) {}

  default void setVelocity(AngularVelocity velocity) {}

  default void stop() {}

  default void resetAngle() {}

  default void resetAngle(Angle angle) {}

  default void setPIDVG(double kP, double kI, double kD, double kV, double kG) {}

  default void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}

  default void setLogKey(String logKey) {}
}
